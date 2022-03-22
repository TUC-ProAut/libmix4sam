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
 */
#include <libmix4sam/registration/RadarDopplerFactor.h>
#include <gtsam/base/Testable.h>

namespace libmix4sam {

gtsam::NonlinearFactor::shared_ptr RadarDopplerFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this))); 
}

void RadarDopplerFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s << "RadarDopplerFactor("
      << keyFormatter(this->key()) << ")\n";
  std::cout << boost::format("  Doppler Measurement:            %0.3d\n") % this->m_doppler_; 
  std::cout << boost::format("  Angle Measurement:              %0.3d\n") % this->m_phi_; 
  std::cout << boost::format("  Time difference between states: %0.3d\n") % this->dT_; 
  std::cout << "  Doppler noise model: \n";  this->dopplerNoisemodel_->print("");
  std::cout << "  Angle noise model: \n"; this->phiNoisemodel_->print("");  
  std::cout << "  Calibration (BodySensor) T_BS: \n"; this->T_BS_->print(""); 
}

bool RadarDopplerFactor::equals(const NonlinearFactor &expected, double tol) const {
  const This *e = dynamic_cast<const This *>(&expected);
  return e != NULL && Base::equals(*e, tol);
      //  &&
      // this->m_doppler_.equals(e->m_doppler_, tol) &&
      // this->m_phi_.equals(e->m_phi_, tol) &&
      // this->s_doppler_->equals(*e->s_doppler_, tol) &&
      // this->s_phi_->equals(*e->s_phi_, tol);
}

boost::shared_ptr<gtsam::GaussianFactor> RadarDopplerFactor::linearize(const gtsam::Values& x) const {
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

double RadarDopplerFactor::error(const Values& c) const {
  if (active(c)) {
    const Vector b = unwhitenedError(c);
    // V4 return 0.5 * this->noiseModel(this->x1(c))->distance(b);
    const gtsam::SharedNoiseModel nM = this->noiseModel(this->x1(c)); // V4
    return nM->loss(nM->squaredMahalanobisDistance(b)); // V4
  } else {
    return 0.0;
  }
};

const gtsam::SharedNoiseModel RadarDopplerFactor::noiseModel(const gtsam::Point2& x1) const {
  gttic_(dopplerNoiseModel);
  
  double s_phi = this->phiNoisemodel_->sigmas()(0);  // Doppler has only one dimension!
  
  double s_phi_hat;
  if(this->T_BS_){   // include Radar extrinsic calibration, if given
    s_phi_hat = pow( (x1(0)-x1(1)*this->T_BS_->y())*sin(m_phi_ + this->T_BS_->theta())-x1(1)*this->T_BS_->x()*cos(m_phi_ + this->T_BS_->theta()), 2.0) * pow(s_phi, 2.0);
  } else {           // Without calibration
    s_phi_hat = pow(x1(0)*sin(m_phi_), 2.0) * pow(s_phi, 2.0); 
  }

  // We don't use the pure measurement, but multiply it by delta t, which can also be seen as measurement with additional uncertainty.
  double d_z__d_v = dT_;
  double d_z__d_t = m_doppler_;
  gtsam::Matrix J(1,2); J << d_z__d_v, d_z__d_t;

  if (this->isDopplerMixModel_){
    const libmix4sam::noiseModelNew::MixBase* p = dynamic_cast<const libmix4sam::noiseModelNew::MixBase*> (&*this->dopplerNoisemodel_);
    if (p == NULL) throw std::invalid_argument( "RadarDopplerFactor: expected a mixture model!"); // This should not happen, because of check in constructor!
    libmix4sam::Mixture mix(p->mixture());
    for (size_t i=0;i<mix.size();i++){
      gtsam::noiseModel::Gaussian* p2 = dynamic_cast<gtsam::noiseModel::Gaussian*> (&*mix.at(i).noiseModel_);
      if (p2 == NULL) throw std::invalid_argument("RadarDopplerFactor: given mixture is not completely Gaussian!");
      // Convert from measurement into translation space (measurement is in m/s and translation space in m)
      gtsam::Matrix P(2,2); P << pow(p2->sigmas()(0), 2.0), 0, 0, pow(p2->sigmas()(1), 2.0);
      double cov_combined = (J*P*J.transpose())(0) + s_phi_hat;
      mix.at(i).noiseModel_ = gtsam::noiseModel::Isotropic::Variance(1, cov_combined);
    }
    return p->cloneWithNewMix(mix);
  }else{
    // Convert from measurement into translation space (measurement is in m/s and translation space in m)
    gtsam::Matrix P(2,2); P << pow(this->dopplerNoisemodel_->sigmas()(0),2.0), 0, 0, pow(this->dopplerNoisemodel_->sigmas()(1),2.0);
    double cov_combined = (J*P*J.transpose())(0) + s_phi_hat;
    //double cov_combined = pow(this->dopplerNoisemodel_->sigmas()(0) * dT_, 2.0) + s_phi_hat;
    return gtsam::noiseModel::Isotropic::Variance(1, cov_combined);
  }
}

Vector RadarDopplerFactor::unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H) const {
  if(this->active(x)) {
    if(H) {
      return evaluateError(this->x1(x), (*H)[0]);
    } else {
      return evaluateError(this->x1(x));
    }
  } else {
    return Vector::Zero(this->dim());
  }
}

Vector RadarDopplerFactor::evaluateError(const gtsam::Point2& x1, boost::optional<Matrix&> H) const {
  double hx;
  if(this->T_BS_){                                                             // include Radar extrinsic calibration, if given
    double alpha = this->T_BS_->theta();
    double xS = this->T_BS_->x();
    double yS = this->T_BS_->y();
    hx = (x1(1)*yS - x1(0)) * cos(m_phi_ + alpha) - x1(1)*xS*sin(m_phi_ + alpha); 
    if (H) {                                                                   // Calculate derivatives if needed
      double d_hx__d_x1 = -cos(m_phi_ + alpha);                                // Longitudinal Translation
      double d_hx__d_x2 = yS*cos(m_phi_ + alpha) - xS*sin(m_phi_ + alpha);     // Theta
      (*H) = (Matrix(1,2)<< d_hx__d_x1, d_hx__d_x2).finished();
    }
  } else {                                                                     // Without calibration
    hx = -x1(0) * cos(m_phi_); 
    if (H) {                                                                   // Calculate derivatives if needed
      // [d_hx__d_x1 d_hx__d_x2]
      (*H) = (Matrix(1,2)<< -cos(m_phi_), 0).finished();
    }
  }

  return (Vector(1) << hx - m_doppler_ * dT_).finished();
}

}