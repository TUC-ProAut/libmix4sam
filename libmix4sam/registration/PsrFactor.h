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
 * @file PsrFactor.h
 * @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief Point Set Registration Factor.
 */
#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <libmix4sam/robust/NoiseModelNew.h>

// for using the matrix logarithm:
#include <unsupported/Eigen/MatrixFunctions>

// For boost std::vector serialization
#include <boost/serialization/vector.hpp>

#include <gtsam/base/timing.h>
//#define gttic_(label) ((void)0)

using namespace gtsam;

namespace libmix4sam {

/**
 * @brief General class template for holding the two point sets and noise models.
 * 
 * @tparam VALUE Should be of type Point3 or Point2
 */
template<class VALUE>
class PsrFactor{
  private:
    typedef PsrFactor<VALUE> This;

  protected:

    /// The second point measurement in the current frame. Will be transformed into reference for error calculation. 
    VALUE m_current_;

    /** The first point measurement in the reference (fixed) frame. 
     * Can be zero, if correspondence is unknown and a GMM noise model for reference pointset is given. 
     */
    VALUE m_reference_;
    
    /// Primary NoiseModel is given for reference points! -> solves Point2Distribution
    gtsam::SharedNoiseModel referenceNoisemodel_;
    /// If true, the problem is P2D or D2D / If false, the problem is simple P2P
    bool isReferenceMixModel_;

    /// Secondary NoiseModel is optional for current Point -> solves Distribution2Distribution according to L2 distance, see Stoyanov et al. 2012
    gtsam::SharedNoiseModel currentNoisemodel_;

    bool isConstNoiseModel_;
    gtsam::SharedNoiseModel constNoiseModel_;

    /// The current noise needs to be rotated to calculate a correct L2 distance, for performance we cache the used rotation (see getUpdatedNoisemodel).
    mutable gtsam::Matrix cachedCovarRot_;

    /// Is the last used noisemodel, see getUpdatedNoisemodel.
    mutable gtsam::SharedNoiseModel cachedNoiseModel_;

  public:
    
    /** default constructor - only use for serialization */
    PsrFactor(){}

    PsrFactor(const VALUE &measuredCurrent, const VALUE &measuredReference,
                const SharedNoiseModel &referenceNoise = nullptr, const SharedNoiseModel &currentNoise = nullptr): 
                m_current_(measuredCurrent), m_reference_(measuredReference), referenceNoisemodel_(referenceNoise), currentNoisemodel_(currentNoise), isConstNoiseModel_(false) {
        
      cachedCovarRot_ = gtsam::Matrix::Identity(traits<VALUE>::dimension,traits<VALUE>::dimension);
      gtsam::SharedNoiseModel cachedNoiseModel_ = nullptr;

      // Do some checks
      const libmix4sam::noiseModelNew::MixBase* p = dynamic_cast<const libmix4sam::noiseModelNew::MixBase*> (&*referenceNoise);
      if (p == NULL) isReferenceMixModel_ = false; else isReferenceMixModel_ = true;

      // if (!this->isReferenceMixModel_){
      //   // Doppler has only one dimension, so only isotropic/diagonal noise models are supported!
      //   const gtsam::noiseModel::Diagonal* p2 = dynamic_cast<const gtsam::noiseModel::Diagonal*> (&*referenceNoise);
      //   if (p2 == NULL) throw std::invalid_argument( "RadarDopplerFactor: doppler noisemodel has either to be unit or an mix model!");
      // }

    }

    /** get the dimension of the factor (number of rows on linearization) */
    virtual size_t dim() const {return traits<VALUE>::dimension;};

    /**
     * @brief Add the currentNoisemodel to the referenceNoisemodel, if currentNoisemodel is present.
     * The method should be used, in case of distribution to distribution matching.
     * You can use this method, if only small rotation changes between frames are expected and the combined noise model
     * has not to be updated at every linearization point.
     * If you expect high rotation changes, the combined noise model should be updated at every linearization point and
     * error calculation. Then, don't call this method in your constructor and use getUpdatedNoisemodel instead!
     * 
     * @return gtsam::SharedNoiseModel 
     */
    gtsam::SharedNoiseModel initCombinedNoiseModel();

    bool isNoiseModelRecalculationNeeded(const gtsam::Matrix& D_hx_pnt) const {
      
      if (this->cachedNoiseModel_){
        // Calculate the geodesic distance between the cached and current rotation matrix
        // (Geodesic distance as distance metric for rotation changes in 3D.
        //  Was used in:
        //    F. Pomerleau et.al, "Comparing ICP variants on real-world data sets", Auton Robot, 2013.
        //  And described originaly in:
        //    D. Q. Huynh, "Metrics for 3D Rotations: Comparison and Analysis", J Math Imaging Vis, 2009.
        // siehe auch: https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation

        gtsam::Matrix dR = D_hx_pnt * this->cachedCovarRot_.transpose();
        double d = dR.log().norm();
        // ALTERNATIV
        //double theta = acos(((D_hx_pnt * this->cachedCovarRot_.transpose()).trace()-1.0)/2.0);
        //double d = 0;
        //if (theta!=0.0)
        //  d = (theta/(2.0*sin(theta)) * (dR - dR.transpose())).norm();
        
        //std::cout << d << std::endl;
        //gtsam::print(D_hx_pnt,"D_hx_pnt: ");
        //gtsam::print(this->cachedCovarRot_,"cachedCovarRot_: ");

        // Use the old noise model, if distance is small
        if (d < 0.09){ // if angle between 
          return false;
        }
      }

      // Default: recalculate
      return true;
    }

    // Rotate current Noise accordingly
    bool rotateCurrentNoise(const gtsam::Matrix& D_hx_pnt, gtsam::Matrix &rotatedCovariance) const {
      gttic_(rotateCurrentNoise);
      const gtsam::noiseModel::Gaussian* p = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*this->currentNoisemodel_);
      if(p){
        rotatedCovariance = D_hx_pnt * p->covariance() * D_hx_pnt.transpose();
        return true;
      }
      std::cout << "RadarFactor:getUpdatedNoiseModel could not rotate covariance matrix for current target!" << std::endl;
      return false;
    }

    const gtsam::SharedNoiseModel updateNoiseModel(const gtsam::Matrix& D_hx_pnt, bool useCached = true) const {

      Matrix rotatedCovariance;
      if (!rotateCurrentNoise(D_hx_pnt, rotatedCovariance)) return this->referenceNoisemodel_;

      if (this->isReferenceMixModel_){
        // Get current noise model as mixture model
        gttic_(addCurrentNoiseToMixture);
        const libmix4sam::noiseModelNew::MixBase *p = dynamic_cast<const libmix4sam::noiseModelNew::MixBase *>(&*this->referenceNoisemodel_);
        if (p) {
          libmix4sam::Mixture mix(p->mixture());
          mix.addToAll(rotatedCovariance);
          //libmix4sam::MixtureClustered cluster(mix);
          //cluster.simplify();
          //libmix4sam::noiseModelNew::MixBase::shared_ptr noiseModel = p->cloneWithNewMix(cluster.getAsMixture());
          //libmix4sam::noiseModelNew::MixBase::shared_ptr noiseModel = p->cloneWithNewMix(mix);
          //return noiseModel;
          if (useCached){
            this->cachedNoiseModel_ = p->cloneWithNewMix(mix); // Cache the current mixture (noise) model
            this->cachedCovarRot_ = D_hx_pnt;                  // Cache the used rotation matrix in D_hx_pnt
            return this->cachedNoiseModel_;
          }else{
            return p->cloneWithNewMix(mix);
          }
        }
      }else{
        // Get current noise model as gaussian in case of known correspondences
        const gtsam::noiseModel::Gaussian* p = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*this->referenceNoisemodel_);
        if (p) {
          gtsam::noiseModel::Gaussian::shared_ptr noiseModel = gtsam::noiseModel::Gaussian::Covariance(rotatedCovariance + p->covariance());
          return noiseModel;
        }
      }

      std::cout << "PsrFactor:updateCachedNoiseModel could not update reference noise model with current model!" << std::endl;
      return this->referenceNoisemodel_;
    }

    const gtsam::SharedNoiseModel noiseModel() const {
      if (!this->isConstNoiseModel_) throw std::invalid_argument( "You asked for noise model without a transformation. This is only possible, if a constant noise model is used!");
      return this->constNoiseModel_;
    }

    /// Calculate and return the noise model using the current state estimation, e.g. current linearization point.
    const gtsam::SharedNoiseModel noiseModel(const gtsam::Pose2& T_B0B1) const {
      gttic_(PsrFactor_noiseModel2d);

      Matrix D_hx_pnt; // Derivation after Point (Should be the rotation matrix!)
      T_B0B1.transformFrom(m_current_, boost::none, D_hx_pnt); // h(x) Point from current frame as seen in reference frame.

      // Do we need to update the noiseModel or can we use the cached one, because only minor changes occurred?
      if (!isNoiseModelRecalculationNeeded(D_hx_pnt)) {return this->cachedNoiseModel_;}

      return updateNoiseModel(D_hx_pnt);
    }

    const gtsam::SharedNoiseModel noiseModel(const gtsam::Point2& x_theta) const {
      gttic_(PsrFactor_noiseModelRadar);

      Pose2 T_B0B1(Rot2(x_theta(1)), Point2(x_theta(0),0));
      
      Matrix D_hx_pnt; // Derivation after Point (Should be the rotation matrix!)
      T_B0B1.transformFrom(m_current_, boost::none, D_hx_pnt); // h(x) Point from current frame as seen in reference frame.

      // Do we need to update the noiseModel or can we use the cached one, because only minor changes occurred?
      if (!isNoiseModelRecalculationNeeded(D_hx_pnt)) {return this->cachedNoiseModel_;}

      return updateNoiseModel(D_hx_pnt);
    }

    /// Calculate and return the noise model using the current state estimation, e.g. current linearization point.
    const gtsam::SharedNoiseModel noiseModel(const gtsam::Pose3& T_B0B1) const {
      gttic_(PsrFactor_noiseModel3d);

      Matrix D_hx_pnt; // Derivation after Point (Should be the rotation matrix!)
      T_B0B1.transformFrom(m_current_, boost::none, D_hx_pnt); // h(x) Point from current frame as seen in reference frame.

      // Do we need to update the noiseModel or can we use the cached one, because only minor changes occurred?
      if (!isNoiseModelRecalculationNeeded(D_hx_pnt)) {return this->cachedNoiseModel_;}

      return updateNoiseModel(D_hx_pnt);
    }

    // Don't update the noise model every time
    void useConstNoiseModel(const gtsam::Pose2& T_B0B1){
      Matrix D_hx_pnt; // Derivation after Point (Should be the rotation matrix!)
      T_B0B1.transformFrom(m_current_, boost::none, D_hx_pnt); // h(x) Point from current frame as seen in reference frame.
      this->constNoiseModel_ = updateNoiseModel(D_hx_pnt, false);
      this->isConstNoiseModel_ = true;
    }
    
    // Don't update the noise model every time
    void useConstNoiseModel(const gtsam::Pose3& T_B0B1){
      Matrix D_hx_pnt; // Derivation after Point (Should be the rotation matrix!)
      T_B0B1.transformFrom(m_current_, boost::none, D_hx_pnt); // h(x) Point from current frame as seen in reference frame.
      this->constNoiseModel_ = updateNoiseModel(D_hx_pnt, false);
      this->isConstNoiseModel_ = true;
    }

    /** print */
    virtual void print(const std::string& s = "") const {
      if (!s.empty()) std::cout << s << "\n";
      traits<VALUE>::Print(this->m_current_,"  current mean: ");
      std::cout << "  current noise model: \n";
      this->currentNoisemodel_->print("");
      traits<VALUE>::Print(this->m_reference_,"  reference mean: ");
      std::cout << "  reference noise model: \n";
      this->referenceNoisemodel_->print();
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const{
      const This *e = dynamic_cast<const This *>(&expected);
      return e != NULL && 
          traits<VALUE>::Equals(this->m_current_, e->m_current_, tol) &&
          traits<VALUE>::Equals(this->m_reference_, e->m_reference_, tol) &&
          this->referenceNoisemodel_->equals(*e->referenceNoisemodel_, tol) &&
          this->currentNoisemodel_->equals(*e->currentNoisemodel_, tol);
    }

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(m_current_);
      ar & BOOST_SERIALIZATION_NVP(m_reference_);
      ar & BOOST_SERIALIZATION_NVP(referenceNoisemodel_);
      ar & BOOST_SERIALIZATION_NVP(currentNoisemodel_);
    }  
};


template<class STATE, class VALUE>
class PsrFactor1: public NonlinearFactor, public PsrFactor<VALUE>{

  private:

    typedef PsrFactor1<STATE, VALUE> This;
    typedef NonlinearFactor Base;
    typedef PsrFactor<VALUE> PsrBase;

  public:

    // typedefs for value types pulled from keys
    typedef STATE X;

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<This> shared_ptr;

    /** default constructor - only use for serialization */
    PsrFactor1() {}

    /** Constructor */
    PsrFactor1(Key key, const VALUE &measuredCurrent, const VALUE &measuredReference,
                const SharedNoiseModel &referenceNoise = nullptr, const SharedNoiseModel &currentNoise = nullptr): 
                Base(KeyVector{key}), 
                PsrBase(measuredCurrent, measuredReference, referenceNoise, currentNoise) {
    };

    virtual ~PsrFactor1() {}

    /// Get the current state's key index connected to this factor.
    inline Key key() const { return keys_[0]; }

    /// Get the current state's value connected to this factor as correct type.
    template<class Q = STATE>
    typename std::enable_if<std::is_same<Q, gtsam::Point2>::value, X>::type x1(const Values& x) const {
      // For the Point2 template variant we need to check if its a vector in case we are calling from Matlab
      // of if it is a Point2 class, if we are using only C++ examples.
      {
        // Test for Point2
        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Point2> *>(&(x.at(keys_[0])));
        if (p) return p->value();
      }
      {
        // Test for Vector
        auto p = dynamic_cast<const gtsam::GenericValue<gtsam::Vector> *>(&(x.at(keys_[0])));
        if (p) return gtsam::Point2(p->value());
      }
      std::cout << "THIS SHOULD NOT HAPPEN!" << std::endl;
      return gtsam::Point2(0,0);
    }
    template<class Q = STATE>
    typename std::enable_if<!std::is_same<Q, gtsam::Point2>::value, const X&>::type x1(const Values& x) const {
      // For Pose2 and Pose3 as state, we can use the simple cast variant.
      return (x.at(keys_[0])).cast<X>();
    } 

    /** get the dimension of the factor (number of rows on linearization) */
    virtual size_t dim() const override {return traits<VALUE>::dimension;};

    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "PsrFactor1<Statetype: " 
          << demangle(typeid(STATE).name()) << ", Valuetype: "  << demangle(typeid(VALUE).name()) << ">("
          << keyFormatter(this->key()) << ")\n";
      PsrBase::print();
    }

    bool equals(const NonlinearFactor &expected, double tol) const override {
      const This *e = dynamic_cast<const This *>(&expected);
      return e != NULL && Base::equals(*e, tol) && PsrBase::equals(*e, tol);
    }

    /**
     * @brief Implement a linearize  method to update the noise model each time.
     * This is necessary to account for a measurement noise model, which depends on the current 
     * state estimation.
     * 
     * @return Linearized factor with noise model.
     */
    boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& x) const override {
      // Only linearize if the factor is active
      if (!active(x)) return boost::shared_ptr<JacobianFactor>();

      // Call evaluate error to get Jacobians and RHS vector b
      std::vector<Matrix> A(size());
      Vector b = -unwhitenedError(x, A);

      this->noiseModel(this->x1(x))->WhitenSystem(A, b);

      // Fill in terms, needed to create JacobianFactor below
      std::vector<std::pair<Key, Matrix> > terms(size());
      for (size_t j = 0; j < size(); ++j) {
        terms[j].first = keys()[j];
        terms[j].second.swap(A[j]);
      }

      return GaussianFactor::shared_ptr(new JacobianFactor(terms, b));
    }

    /**
     * Implementation of the error calculation for the 2D case. 
     * "If the optional Matrix reference argument is specified, it should compute both the function
     *  evaluation and its derivative in X." CITE from GtSAM
     **/
    Vector evaluateError(const gtsam::Pose2& T_B0B1, boost::optional<Matrix&> H = boost::none) const{
      gttic_(Psr2DPriorFactor_evaluateError);

      Matrix D_hx_B0B1; // Derivation after transformation matrix
      gtsam::Point2 hx = T_B0B1.transformFrom(this->m_current_,
         (H) ? boost::optional<Matrix&>(D_hx_B0B1) : boost::none); // h(x) Point from current frame as seen in reference frame.

      // Calculate derivatives if needed
      if (H) *H = D_hx_B0B1;

      return hx - this->m_reference_;
    }

    /**
     * Implementation of the error calculation for the 3D case. 
     * "If the optional Matrix reference argument is specified, it should compute both the function
     *  evaluation and its derivative in X." CITE from GtSAM
     **/
    Vector evaluateError(const gtsam::Pose3& T_B0B1, boost::optional<Matrix&> H = boost::none) const{
      gttic_(Psr2DPriorFactor_evaluateError);

      Matrix D_hx_B0B1; // Derivation after transformation matrix
      gtsam::Point3 hx = T_B0B1.transformFrom(this->m_current_,
         (H) ? boost::optional<Matrix&>(D_hx_B0B1) : boost::none); // h(x) Point from current frame as seen in reference frame.

      // Calculate derivatives if needed
      if (H) *H = D_hx_B0B1;

      return hx - this->m_reference_;
    }

    /**
     * Implementation of the error calculation for the radar scenario with only 1D velocity and 1D rotation. 
     * "If the optional Matrix reference argument is specified, it should compute both the function
     *  evaluation and its derivative in X." CITE from GtSAM
     **/
    Vector evaluateError(const gtsam::Point2& x_theta, boost::optional<Matrix&> H = boost::none) const{
      gttic_(PsrRadarPriorFactor_evaluateError);

      Rot2 R_FM(x_theta(1));
      Matrix D_hx_theta;
      gtsam::Point2 hx = R_FM.rotate(this->m_current_, (H) ? boost::optional<Matrix&>(D_hx_theta) : boost::none );
      hx = hx + Point2(x_theta(0),0);

      // Calculate derivatives if needed
      if (H) {
        Matrix D_hx_x(2,1);
        D_hx_x << 1, 0;
        (*H) = (Matrix(2,2)<< D_hx_x, D_hx_theta).finished();
      }

      return hx - this->m_reference_;
    }

    /** Calls the 1-key specific version of evaluateError.
     */
    virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const {
      if(this->active(x)) {
        const X& x1 = x.at<X>(keys_[0]);
        if(H) {
          return evaluateError(x1, (*H)[0]);
        } else {
          return evaluateError(x1);
        }
      } else {
        return Vector::Zero(this->dim());
      }
    }

    /**
     * @brief Implement the error method.
     * Calculates the error vector considering the updated noise model for the current state estimation.
     * 
     * @param c 
     * @return double 
     */
    virtual double error(const Values& c) const override {
      if (active(c)) {
        const Vector b = unwhitenedError(c);
        // V4 return 0.5 * this->noiseModel(this->x1(c))->distance(b);
        const gtsam::SharedNoiseModel nM = this->noiseModel(this->x1(c)); // V4
        return nM->loss(nM->squaredMahalanobisDistance(b)); // V4
      } else {
        return 0.0;
      }
    };


  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NonlinearFactor",
          boost::serialization::base_object<Base>(*this));
      ar & boost::serialization::make_nvp("PsrBase",
        boost::serialization::base_object<PsrBase>(*this));
    }

};

//class Psr3DPriorFactor: PsrFactor1<gtsam::Pose3, gtsam::Point3>{
//class PsrRadarPriorFactor: PsrFactor1<gtsam::Vector2, gtsam::Point2>{
//class Psr2DTransitionFactor: PsrFactor2<gtsam::Pose2, gtsam::Point2>{
//class Psr3DTransitionFactor: PsrFactor2<gtsam::Pose3, gtsam::Point3>{

} // namespace libmix4sam
