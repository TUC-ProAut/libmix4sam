/***************************************************************************
 * libmix4sam - Mixtures for Smoothing and Mapping Library
 *
 * Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
 * For more information see https://mytuc.org/mix
 *
 * libmix4sam is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libmix4sam is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file testPsr2DFactor.cpp
 * @author Sven Lange (sven.lange@gmail.com)
 * @brief This file implements a simple example to test the robust point set registration in 2D.
 */

#include <iostream>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "libmix4sam/robust/NoiseModelNew.h"
#include "libmix4sam/registration/PsrFactor.h"

typedef libmix4sam::PsrFactor1<gtsam::Pose2, gtsam::Point2> Psr2DPriorFactor;

int main(int argc, char *argv[]) {
  
  gtsam::Values initialEstimate;
  initialEstimate.insert(1, gtsam::Pose2(0,0,0));
  gtsam::NonlinearFactorGraph graph;

  // Define some targets for the previous frame
  std::vector<gtsam::Point2> fixed;
  fixed.push_back(gtsam::Point2(10,1));
  fixed.push_back(gtsam::Point2(20,-1));
  fixed.push_back(gtsam::Point2(30,10));
  fixed.push_back(gtsam::Point2(30,-10));
  
  // Define a ground truth transformation and transform the previous targets into the current frame.
  gtsam::Pose2 T(0.25, -0.25, 0.18);
  T.print("============ Transformation ==============\n");
  std::vector<gtsam::Point2> current;
  for(const auto& lmk: fixed) current.push_back(T.inverse().transformFrom(lmk));

  // Convert the previous frame into a GMM.
  libmix4sam::Mixture refPdf;
  gtsam::SharedNoiseModel refNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.2, 0.3));
  for(const auto& lmk: fixed) 
      refPdf.add( libmix4sam::MixComponent( refNoise, 1.0 / fixed.size(), lmk) );
  
  // Use the GMM to instantiate a robust noise model (here we use MaxSumMix).
  libmix4sam::noiseModelNew::MaxSumMix::shared_ptr refNoiseModel;
  refNoiseModel = libmix4sam::noiseModelNew::MaxSumMix::Create(refPdf);

  // Add each target of current point set to the graph with mixture distribution of the fixed set as noise model.
  gtsam::SharedNoiseModel currNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.2, 0.3));
  for(const auto& lmk: current)
      graph.add( Psr2DPriorFactor(1, lmk, gtsam::Point2(0,0), refNoiseModel, currNoise) );

  // Show the actual graph.
  graph.print("================= GRAPH ==================\n");

  // Optimize
  std::cout << "============== OPTIMIZATION ==============" << std::endl;
  gtsam::LevenbergMarquardtParams optimizerParameters;
  //SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
  optimizerParameters.setVerbosityLM("SUMMARY");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, optimizerParameters);

  // Show the result
  gtsam::Values result = optimizer.optimizeSafely();
  result.print("================= RESULT =================\n");

  // For convenience show the ground truth again
  gtsam::traits<gtsam::Pose2>::Print(T,"Ground truth reference: ");

  return 0;
}
