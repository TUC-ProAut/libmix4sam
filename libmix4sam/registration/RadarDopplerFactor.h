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
 * @file RadarDopplerFactor.h
 * @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief A factor for including the Doppler measurement of an automotive radar sensor.
 */
#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <libmix4sam/robust/NoiseModelNew.h>

using namespace gtsam;

namespace libmix4sam {

//template<class VALUE>
class RadarDopplerFactor: public NonlinearFactor{
  private:

    typedef RadarDopplerFactor This;
    typedef NonlinearFactor Base;

  protected:

    // Measurement of the target's Doppler velocity.
    double m_doppler_;

    // Standard deviation for target's Doppler velocity measurement.
    //double s_doppler_;
    gtsam::SharedNoiseModel dopplerNoisemodel_;
    bool isDopplerMixModel_;

    // Measurement of the target's angle.
    double m_phi_;

    // Standard deviation for target's angle measurement.
    //double s_phi_;
    gtsam::SharedDiagonal phiNoisemodel_;
    
    // Time difference between the current and the last state estimation.
    double dT_;

    // Radar calibration
    boost::optional<gtsam::Pose2> T_BS_; ///< The pose of the sensor in the body frame

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<This> shared_ptr;

    // typedefs for value types pulled from keys
    //typedef gtsam::Vector X;
    typedef gtsam::Point2 X;
    //typedef VALUE X;

    /** default constructor - only use for serialization */
    RadarDopplerFactor() {}

    /** Constructor */
    RadarDopplerFactor(Key key, const double &measuredDoppler, const double &measuredPhi,
                const SharedNoiseModel &dopplerNoise, const SharedDiagonal &phiNoise, const double &deltaT, 
                boost::optional<gtsam::Pose2> T_BS = boost::none): 
                Base(cref_list_of<1>(key)), m_doppler_(measuredDoppler), m_phi_(measuredPhi), 
                dopplerNoisemodel_(dopplerNoise),phiNoisemodel_(phiNoise), dT_(deltaT), T_BS_(T_BS) {

      const libmix4sam::noiseModelNew::MixBase* p = dynamic_cast<const libmix4sam::noiseModelNew::MixBase*> (&*dopplerNoise);
      if (p == NULL) isDopplerMixModel_ = false; else isDopplerMixModel_ = true;

      if (!this->isDopplerMixModel_){
        // Doppler has only one dimension, so only isotropic/diagonal noise models are supported!
        const gtsam::noiseModel::Diagonal* p2 = dynamic_cast<const gtsam::noiseModel::Diagonal*> (&*dopplerNoise);
        if (p2 == NULL) throw std::invalid_argument( "RadarDopplerFactor: doppler noisemodel has either to be unit or an mix model!");
      }
    };

    virtual ~RadarDopplerFactor() {}

    /** get the dimension of the factor (number of rows on linearization) */
    virtual size_t dim() const {return 1;};

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const;

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const;

    /** implement functions needed for NonlinearFactor */

    /**
     * @brief Implement a linearize  method.
     * This is necessary to account for a measurement noise model, which depends on the current 
     * state estimation.
     * 
     * @return Linearized factor with noise model.
     */
    virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& x) const;

    /**
     * @brief Implement the error method.
     * Calculates the error vector considering the updated noise model for the current state estimation.
     * 
     * @param x 
     * @return double 
     */
    virtual double error(const Values& c) const;

    /** implement additional functions needed error() and linearize() */

    /// Get the current state's key index connected to this factor.
    inline Key key() const { return keys_[0]; }

    /// Get the current state's value connected to this factor.
    //inline const X& x1(const Values& x) const {return x.at<X>(keys_[0]);};
    //inline const gtsam::Vector& x1(const Values& x) const {return (x.at(keys_[0])).cast<gtsam::Vector>();};
    
    //inline const X& x1(const Values& x) const {return (x.at(keys_[0])).cast<X>();};

    X x1(const Values& x) const {
      // For the Point2 template variant we need to check if its a vector in case we are calling from Matlab
      // or if it is a Point2 class, if we are using only C++ examples.
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
    
    /// Calculate and return the noise model using the current state estimation, e.g. current linearization point.
    //const gtsam::SharedNoiseModel noiseModel(const Vector& x1) const;
    const gtsam::SharedNoiseModel noiseModel(const gtsam::Point2& x1) const;
    //const SharedDiagonal noiseModel(const Pose2& x1) const;

    /// Calls the 1-key specific version of evaluateError.
    virtual Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const;

    // For matlab wrapper it is good to have a seperated error method to call with specific state.
    // Additionally, we could also have an evaluateError method for other state variable definition than Vector2
    //Vector evaluateError(const Vector& x1, boost::optional<Matrix&> H = boost::none) const;
    Vector evaluateError(const gtsam::Point2& x1, boost::optional<Matrix&> H = boost::none) const;
    //Vector evaluateError(const Pose2& x, boost::optional<Matrix&> H = boost::none) const;
  
  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NonlinearFactor",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(m_doppler_);
      ar & BOOST_SERIALIZATION_NVP(m_phi_);
      ar & BOOST_SERIALIZATION_NVP(dopplerNoisemodel_);
      ar & BOOST_SERIALIZATION_NVP(phiNoisemodel_);
      ar & BOOST_SERIALIZATION_NVP(isDopplerMixModel_);
      ar & BOOST_SERIALIZATION_NVP(dT_);
      ar & BOOST_SERIALIZATION_NVP(T_BS_);
    }
};


}