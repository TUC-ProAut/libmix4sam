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
 * @file     libmix4sam.h
 * @brief    Wrapper interface file.
 */

// This is an interface file for automatic MATLAB wrapper generation.  See
// gtsam.h for full documentation and more examples.

// Define available external classes from GTSAM
// of course, the gtsam matlab wrapper has to be within matlab's search path for this to work.
class gtsam::Pose2;
class gtsam::Pose3;
virtual class gtsam::Rot3;
virtual class gtsam::Point3;
virtual class gtsam::Point2;
virtual class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NoiseModelFactor;


#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

namespace libmix4sam {

  /************************************************************************************/
  /* MIXTURE NOISE MODELS */
  #include <libmix4sam/robust/NoiseModelNew.h>
  #include <gtsam/linear/NoiseModel.h>

  class MixComponent {
    MixComponent(gtsam::noiseModel::Base* noiseModel, double w, Vector mu);
    MixComponent(gtsam::noiseModel::Base* noiseModel, double w);
    gtsam::noiseModel::Base* noiseModel() const;
    double w() const;
    Vector mu() const;
    double bhattacharyya(const libmix4sam::MixComponent &other) const;
    void print(string s) const;
  };

  class Mixture {
    Mixture();
    libmix4sam::MixComponent at(size_t n) const;
    size_t size() const;
    void add(const libmix4sam::MixComponent& c);
    void add(const libmix4sam::Mixture& other, double influence);
    void addToAll(const gtsam::noiseModel::Base* g);
    Vector getScalingFactors() const;
    Matrix getWhitenedComponents(Vector& v) const;
    Vector getExponents(Vector &unwhitenedError) const;
    Vector w() const;
    libmix4sam::MixComponent merge() const;
    void print(string s) const;
  };

  class MixtureClustered {
    MixtureClustered(const libmix4sam::Mixture& mix);
    pair<Vector, Matrix> distBhattacharyya() const;
    void merge(size_t index1, size_t index2);
    bool simplify(double threshold);
    libmix4sam::Mixture getAsMixture() const;
    void print(string s) const;
  };

  namespace noiseModelNew {

    virtual class MixBase : gtsam::noiseModel::Base {
      libmix4sam::Mixture mixture() const;
      void print(string s) const;
    };

    virtual class MaxMix : libmix4sam::noiseModelNew::MixBase {
      static libmix4sam::noiseModelNew::MaxMix* Create(const libmix4sam::Mixture& noise);

      void print(string s) const;

      double squaredMahalanobisDistance(Vector& v) const;
      double distanceNormalizer(Vector& v) const;

      Vector whiten(Vector& v) const;
      Matrix Whiten(Matrix& A, Vector& v) const;
      pair<double,Vector> regularWhite(Vector& v) const;

      // enabling serialization functionality
      //void serializable() const;
    };  

    virtual class MaxSumMix : libmix4sam::noiseModelNew::MaxMix {
      static libmix4sam::noiseModelNew::MaxSumMix* Create(const libmix4sam::Mixture& noise);

      void print(string s) const;

      double squaredMahalanobisDistance(Vector& v) const;

      Vector whiten(Vector& v) const;
      //double distanceNormalizer(Vector& v) const;

      //Vector whiten(Vector& v) const;
      Matrix Whiten(Matrix& A, Vector& v, size_t idx_max) const;
      //pair<double,Vector> regularWhite(Vector& v) const;

      // enabling serialization functionality
      //void serializable() const;
    };  


    virtual class SumMix : libmix4sam::noiseModelNew::MixBase {
      static libmix4sam::noiseModelNew::SumMix* Create(const libmix4sam::Mixture& noise);

      void print(string s) const; 

      double squaredMahalanobisDistance(Vector& v) const;

      Vector whiten(Vector& v) const;
      Matrix Whiten(Matrix& A, Vector& v) const;
      Vector getScalings() const;
      //pair<double,Vector> regularWhite(Vector& v) const;

      // enabling serialization functionality
      //void serializable() const;
    };  

  } //end namespace noiseModelNew

  /************************************************************************************/
  /* REGISTRATION FACTORS */

  #include <libmix4sam/registration/PsrFactor.h>
  template<STATE, VALUE>
  virtual class PsrFactor1: gtsam::NonlinearFactor{
    PsrFactor1();
    PsrFactor1(size_t key, const VALUE &measuredCurrent, const VALUE &measuredReference, const gtsam::noiseModel::Base* referenceNoise);
    PsrFactor1(size_t key, const VALUE &measuredCurrent, const VALUE &measuredReference, const gtsam::noiseModel::Base* referenceNoise, const gtsam::noiseModel::Base* currentNoise);
    void print(string s) const;
    // enabling serialization functionality
    void serialize() const;
  };
  typedef libmix4sam::PsrFactor1<gtsam::Pose2, gtsam::Point2> Psr2DPriorFactor;
  typedef libmix4sam::PsrFactor1<gtsam::Pose3, gtsam::Point3> Psr3DPriorFactor;
  typedef libmix4sam::PsrFactor1<gtsam::Point2, gtsam::Point2> PsrRadarPriorFactor;


} // end namespace libmix4sam

/************************************************************************************/
/* Add additional functionality into gtsam namespace */
namespace gtsam {

  #include <gtsam/base/timing.h>
  void tictoc_print_();
  void tictoc_reset_();

  #include <gtsam/base/Lie.h>
  //template<T = {gtsam::Pose2, gtsam::Pose3}> 
  //interpolate(const T &T1, const T &T2, const double t);
  gtsam::Pose3 interpolate(const gtsam::Pose3 &T1, const gtsam::Pose3 &T2, const double t);
  gtsam::Pose2 interpolate(const gtsam::Pose2 &T1, const gtsam::Pose2 &T2, const double t);

}
/************************************************************************************/

