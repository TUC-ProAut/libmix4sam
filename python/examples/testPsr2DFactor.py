"""Robust 2D point-set registration example.

Usage:
    ./testPsr2DFactor.py

Author:
    Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)

License:
    This file is part of
    libmix4sam - Mixtures for Smoothing and Mapping Library

    Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
    For more information see https://mytuc.org/mix

    libmix4sam is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libmix4sam is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.

    Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)
"""
import gtsam
import libmix4sam
import numpy as np

initialEstimate = gtsam.Values()
initialEstimate.insert(1, gtsam.Pose2(0,0,0))

# Define some targets for the previous frame
fixed = []
fixed.append(gtsam.Point2(10,1))
fixed.append(gtsam.Point2(20,-1))
fixed.append(gtsam.Point2(30,10))
fixed.append(gtsam.Point2(30,-10))

# Define a ground truth transformation and transform the previous targets into the current frame.
T = gtsam.Pose2(0.25,-0.25, 0.18)
T.print("============ Transformation ==============\n")
current = []
for f in fixed:
    current.append(T.inverse().transformFrom(f))
    #print(f"Original Point: {np.array2string(f)}")
    #print(f"Transformed Point: {np.array2string(current[-1], precision=2)}")

# Convert the previous frame into a GMM.
refPdf = libmix4sam.Mixture()
refNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.3]))
for lmk in fixed:
    refPdf.add(libmix4sam.MixComponent(refNoise, 1.0/len(fixed),lmk))

# Use the GMM to instantiate a robust noise model (here we use MaxSumMix).
refNoiseModel = libmix4sam.noiseModelNew.MaxSumMix.Create(refPdf)
currNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.3]))

# Add each target of current point set to the graph with mixture distribution of the fixed set as noise model.
graph = gtsam.NonlinearFactorGraph()
for lmk in current:
    graph.add(libmix4sam.Psr2DPriorFactor(1,lmk,gtsam.Point2(0,0),refNoiseModel,currNoise))

# Show the actual graph.
graph.print("================= GRAPH ==================\n")

# Optimize
optimizerParameters = gtsam.LevenbergMarquardtParams()
#SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
optimizerParameters.setVerbosityLM("SUMMARY")
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate, optimizerParameters)

result = optimizer.optimizeSafely();

# Show the result
result.print("================= RESULT =================\n")

# For convenience show the ground truth again
T.print(     "=========== GROUND TRUTH WAS =============\n")
