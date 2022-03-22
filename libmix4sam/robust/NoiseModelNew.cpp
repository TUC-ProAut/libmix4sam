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
 * @file NoiseModelNew.cpp
 * @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
 */

#include "NoiseModelNew.h"
#include "libmix4sam/robust/numericalRobust.h"
#include <gtsam/base/numericalDerivative.h>

#include <gtsam/base/timing.h>
//#define gttic_(label) ((void)0)

#define SUMMIX_NUMERIC 0
#define MAXSUMMIX_NUMERIC 0
/* MaxSumMix dampening parameter. 
   For details, see:
   Pfeifer, T., Lange, S. and Protzel, P. (2021) ‘Advancing Mixture Models for Least Squares Optimization’, 
   IEEE Robotics and Automation Letters, 6(2), pp. 3941--3948. doi:10.1109/LRA.2021.3067307. */
#define MAXSUMMIX_DAMPENING 10.0

namespace libmix4sam {

  // from gtsam's NonlinearFactor.cpp
  static void check(const gtsam::noiseModel::Base::shared_ptr& noiseModel, size_t m) {
    if (noiseModel && m != noiseModel->dim())
      throw std::invalid_argument(
        boost::str(boost::format(
          "NoiseModel: NoiseModel has dimension %1% instead of %2%.") % noiseModel->dim() % m));
  }

  MixComponent::MixComponent(const gtsam::noiseModel::Base::shared_ptr noiseModel, double w, gtsam::Vector mu)  :
    noiseModel_(noiseModel), w_(w) {

    check(this->noiseModel_, mu.rows());
    this->mu_ = mu;
  };

  void MixComponent::print(const std::string& name) const{
    std::cout << name;
    std::cout << boost::str(boost::format("w=%1$0.4d ") % this->w());
    static const Eigen::IOFormat matlab( Eigen::StreamPrecision, // precision
      0, /* flags */ " ", /* coeffSeparator */ ";\n", /* rowSeparator */
      "",  /* rowPrefix */ "", /* rowSuffix */ "[ ", /* matPrefix */ " ]'" /* matSuffix */ );
    std::cout << "mu=" << gtsam::Matrix(this->mu().matrix().transpose()).format(matlab) << " ";
    std::cout << "Noisemodel ";
    noiseModel_->print();
  }

  double MixComponent::bhattacharyya(const MixComponent &other) const{
    const gtsam::noiseModel::Gaussian* n1 = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*this->noiseModel_);
    const gtsam::Matrix cov1 = n1->covariance();
    const gtsam::noiseModel::Gaussian* n2 = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*(other.noiseModel()));
    const gtsam::Matrix cov2 = n2->covariance();

    gtsam::Matrix covMean = (cov1 + cov2) / 2.0;
    gtsam::Matrix meanDiff = this->mu_ - other.mu();
    gtsam::Matrix mahala = 0.125 * meanDiff.transpose() * covMean.inverse() * meanDiff;

    return mahala(0) + 0.5 * log(covMean.determinant() / sqrt(cov1.determinant()*cov2.determinant())); 
  }

  Mixture::Mixture(const MixComponent& c) { 
    this->dim_ = c.noiseModel_->dim();
    push_back(c); 
  }

  Mixture& Mixture::operator&(const MixComponent& component) {
    if (this->dim() > 0) check(component.noiseModel_, this->dim());
    push_back(component);
    this->cacheScalingFactors_ = gtsam::Vector(); // reset scaling factors cache
    return *this;
  }

  bool Mixture::equals(const Mixture& expected, double tol) const {
    return this->size() == expected.size() && std::equal(this->begin(), this->end(), expected.begin());
  }

  void Mixture::print(const std::string& name) const {
    if (!name.empty()) std::cout << name << ":\n";
    for (size_t i=0;i<this->size();i++){
      this->at(i).print(boost::str(boost::format("Mixture component %1%: ") % i));
    }
  }

  void Mixture::add(const MixComponent& c) { 
    if (this->dim() > 0) {
      // Check for same dimensionality!
      check(c.noiseModel(), this->dim());
    }else{
      this->dim_ = c.noiseModel()->dim();
    }
    this->push_back(c); 
    this->cacheScalingFactors_ = gtsam::Vector(); // reset scaling factors cache
  }

  gtsam::Vector Mixture::w() const{
    gtsam::Vector ret(this->size());
    for (size_t i=0;i<this->size();i++) ret(i) = this->at(i).w();
    return ret;
  }

  void Mixture::setWeights(const gtsam::Vector &weights){
    for (size_t i=0;i<weights.size();i++) this->at(i).w_ = weights(i);
    this->cacheScalingFactors_ = gtsam::Vector(); // reset scaling factors cache
  }

  gtsam::Vector Mixture::getScalingFactors() const {
    if (this->cacheScalingFactors_.size() > 0)  // We already calculated the scaling factors!
      return this->cacheScalingFactors_;

    this->cacheScalingFactors_ = gtsam::Vector::Zero(this->size());           // normalization factor for all components
    for (size_t i=0;i<this->size();i++){
      gtsam::noiseModel::Gaussian* p = dynamic_cast<gtsam::noiseModel::Gaussian*> (&*this->at(i).noiseModel_);
      if (p == NULL) throw std::invalid_argument( "Mixture::getScalingFactors: can not convert one of the mixture component to a gaussian!" );
      this->cacheScalingFactors_(i) = this->at(i).w_ * p->R().determinant();
    }
    return this->cacheScalingFactors_;
  };

  gtsam::Matrix Mixture::getWhitenedComponents(const gtsam::Vector& v) const {
    gtsam::Matrix we = gtsam::Matrix::Zero(this->size(),v.size()); // whitened error for all components of mixture
    
    for(size_t i=0; i<this->size();i++){
      // Currently a Gaussian based error model for each mixture component is needed, because of the needed square root information matrix
      const gtsam::noiseModel::Gaussian* p = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*this->at(i).noiseModel_);
      if (p == NULL) throw std::invalid_argument( "Mixture: only gaussian based noise models are currently supported!" );
      we.row(i) = p->whiten(v - this->at(i).mu_); // kommt mu_ auch mit in die Ableitung bzw. den Ergebnisvector a??
    }

    return we;
  };

  Vector Mixture::getExponents(const gtsam::Vector &unwhitenedError) const{
    
    // whitened error for all components of mixture
    gtsam::Matrix we = this->getWhitenedComponents(unwhitenedError);

    gtsam::Vector exponents = gtsam::Vector::Zero(this->size());  
    for(size_t i=0; i<this->size();i++)
      exponents(i) = -0.5 * we.row(i).squaredNorm();

    return exponents;
  }

  std::vector<gtsam::Vector> Mixture::getDExponents(const gtsam::Vector &unwhitenedError) const{

    std::vector<gtsam::Vector> dExponents;   // derivative of exponents (half Mahalanobis norm) for all components of mixture
    for(size_t i=0; i<this->size();i++){

      // Currently a Gaussian based error model for each mixture component is needed, because of the needed square root information matrix
      const gtsam::noiseModel::Gaussian* p = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*this->at(i).noiseModel_);
      if (p == NULL) throw std::invalid_argument( "SumMix: only gaussian based noise models are currently supported!");

      // Note: v and mu are negated, in contrast to whitenedComponents, which is correct!
      dExponents.push_back(p->information() * (this->at(i).mu_ - unwhitenedError));
    }

    return dExponents;
  }

  void Mixture::normalizeWeights(){ 
    this->setWeights((1.0/this->w().sum())*this->w());
    this->cacheScalingFactors_ = gtsam::Vector(); // reset scaling factors cache
  }

  void Mixture::add(const Mixture& other, double influence){
    // Scale down the weights for adding the other components with their weights.
    // The other components' weights should sum up to 1!
    this->normalizeWeights();

    gtsam::Vector otherWeights = other.w();
    // Normalize in case it is not normalized
    otherWeights /= otherWeights.sum();
    // Now take the influence into account
    otherWeights *= influence;
    this->setWeights(this->w() * (1.0-influence));
    
    for (size_t i=0;i<other.size();i++)
      this->push_back(MixComponent(other.at(i).noiseModel(), otherWeights(i), other.at(i).mu()));

    this->cacheScalingFactors_ = gtsam::Vector(); // reset scaling factors cache
  }

  MixComponent Mixture::merge() const {
    // see "EM algorithms for Gaussian mixtures with split-and-merge operation" Zhihua Zhang, Chibiao Chen, Jian Sun,Kap Luk Chan
    double w = this->at(0).w();
    gtsam::Vector mu = this->at(0).w() * this->at(0).mu();
    gtsam::noiseModel::Gaussian* tmp = dynamic_cast<gtsam::noiseModel::Gaussian*> (&*this->at(0).noiseModel_);
    gtsam::Matrix R = this->at(0).w() * tmp->covariance();
    if (this->size() > 1){
      for (size_t i=1;i<this->size();i++){
        // Collect weights
        w += this->at(i).w();
        // Collect means
        mu += this->at(i).w() * this->at(i).mu();
        // Collect covariances
        gtsam::noiseModel::Gaussian* tmp2 = dynamic_cast<gtsam::noiseModel::Gaussian*> (&*this->at(i).noiseModel_);
        R += this->at(i).w() * tmp2->covariance();
      }
    }
    return MixComponent(gtsam::noiseModel::Gaussian::Covariance(R / w), w, mu / w);
  }

  void MixtureClustered::print(const std::string& name) const{
    for (size_type i=0; i<this->size(); i++){
      this->at(i).print(boost::str(boost::format("Cluster %1%: ") % i));
    }
  }

  std::pair<gtsam::Vector, gtsam::Matrix> MixtureClustered::distBhattacharyya() const{
    // Distance between merged mixtures (Clusters)
    size_t n = this->size();
    size_t nPairs = n/2.0*(n-1);
    gtsam::Matrix pairs(nPairs,2);
    gtsam::Vector dist(nPairs);
    size_t idx = 0;
    for (size_t iFirst=0; iFirst < (n-1); iFirst++){
      //std::cout << nPairs << ": " << iFirst << std::endl;
      const MixComponent &first = this->at(iFirst).merge();
      for (size_t iSecond=iFirst+1; iSecond < n; iSecond++){
        //std::cout << iFirst << ", " << iSecond << " idx:" << idx << std::endl;
        pairs.row(idx) << iFirst, iSecond;
        dist(idx++) = first.bhattacharyya(this->at(iSecond).merge());
      }
    }
    return std::make_pair(dist,pairs);
  }

  void MixtureClustered::merge(size_type index1, size_type index2){
    // Create new mixture cluster
    for (size_t i=0; i<this->at(index2).size(); i++){
      this->at(index1).add(this->at(index2).at(i));
    }
    this->erase(this->begin() + index2);
  }

  bool MixtureClustered::simplify(double threshold) {
    gttic_(MixtureClustered_simplify);
    // Calculate the Bhattacharyya distance.
    std::pair<gtsam::Vector, gtsam::Matrix> dist = this->distBhattacharyya();
    // Look for similar clusters
    //Eigen::ArrayXd test = dist.first.array(); test *= -1.0;
    gtsam::Vector distExp = (dist.first * -1.0).array().exp();
    size_t ind = 0;
    double max = distExp.maxCoeff(&ind);
    if (max > threshold){ 
      // Remaining clusters to simplify.
      this->merge(dist.second(ind,0),dist.second(ind,1));
      // Run again until all similar clusters are merged
      if (!this->simplify(threshold)) return false;
    }
    return true;
  }

  Mixture MixtureClustered::getAsMixture() const {
    Mixture ret(this->at(0).merge()); // Create Mixture with Cluster 0
    for (size_type i=1; i<this->size(); i++)
      ret.add(this->at(i).merge());   // Add remaining Clusters to mixture
    return ret;
  }

  Mixture operator&(const MixComponent& c1, const MixComponent& c2) {
    Mixture mix(c1);
    return mix & c2;
  }

  namespace noiseModelNew{

    /* ****************************************************************************** */
    /* MixBase */

    void MixBase::print(const std::string& name) const {
      mixture_.print(name);
    }

    gtsam::Vector MixBase::whiten(const gtsam::Vector& v) const { 
      //std::cout << "whiten" << std::endl;
      Vector r = v; 
      this->whitenInPlace(r); 
      return r; 
    }

    /* ****************************************************************************** */
    /* MaxMix */

    MaxMix::shared_ptr MaxMix::Create(const libmix4sam::Mixture& noise){
      return shared_ptr(new MaxMix(noise));
    }

    bool MaxMix::equals(const Base& expected, double tol) const {
      const MaxMix* p = dynamic_cast<const MaxMix*> (&expected);
      if (p == NULL) return false;
      return mixture_.equals(p->mixture(),tol);
    }

    void MaxMix::print(const std::string& name) const {
      mixture_.print(name);
    }

    void MaxMix::normalizer2dim(Vector &c) const{
      double gamma = c.maxCoeff();
      c = (-2.0 * (c / gamma).array().log()).sqrt();
    }

    size_t MaxMix::min(Vector &c, Matrix &we) const{
      double min_err = this->cdistance(c(0), we.row(0));
      size_t min_idx = 0;
      for(size_t i=1; i<mixture_.size();i++){
        double e = this->cdistance(c(i), we.row(i));
        if (e < min_err){
            min_err = e;
            min_idx = i;
        }
      }

      return min_idx;
    }

    Vector MaxMix::processMixture(const Vector& v, boost::optional<ComponentSelect &> gamma_m) const {
      //std::cout << "processMixture" << std::endl;
      gttic_(MaxMix_processMixture);

      // Speed optimization by using cached results
      double md5sum = this->mixture_.md5();
      if(this->cacheErrorVector_.size() > 0){
        if( ((this->cacheErrorVector_ - v).array().abs() <= 1e-9).all() && (abs(this->cacheMixSum_ - md5sum) <= 1e-9)) {
          if (gamma_m){
            (*gamma_m).second = this->cacheNormalizerValue_;
            (*gamma_m).first = this->cacheNormalizerIdx_;
          }
          return this->cacheWhitenedError_;
        }
      }

      // normalization factor for all components
      gtsam::Vector c = this->mixture_.getScalingFactors();
      // whitened error for all components of mixture       
      gtsam::Matrix we = this->mixture_.getWhitenedComponents(v);

      // Modify the normalization factors to be used as extra dimension in the error vector
      this->normalizer2dim(c);

      // Get index of component with minimum error
      size_t min_idx = this->min(c, we);

      if (gamma_m){
        (*gamma_m).second = c(min_idx);
        (*gamma_m).first = min_idx;
      }
        
      this->cacheWhitenedError_ = we.row(min_idx);
      this->cacheErrorVector_ = v;
      this->cacheNormalizerIdx_ = min_idx;
      this->cacheNormalizerValue_ = c(min_idx);
      this->cacheMixSum_ = md5sum;

      return this->cacheWhitenedError_;  
    }

    
    gtsam::Matrix MaxMix::Whiten(const gtsam::Matrix& A, const size_t& idx) const { 
      //std::cout << "Whiten" << std::endl;
      
      // Get the current mixture component
      const gtsam::noiseModel::Gaussian* p = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*mixture_.at(idx).noiseModel_);
      if (p == NULL) {std::cout << "processMixture: only gaussian based noise models are supported!" << std::endl; return A;}
      
      // As we extended our error vector, we need to add an additional line to the A matrix!
      //gtsam::Matrix C = Matrix::Zero(A.rows()+1, A.cols());

      // TODO: Only if we want to optimize over the standard deviations or the weights of the components, we need to add the
      // extra dimension of the error vector and accordingly the extra entries in the A matrix
      gtsam::Matrix C = Matrix::Zero(A.rows(), A.cols());
      
      // Whiten matrix by current mixture component
      C.topLeftCorner(A.rows(), A.cols()) = p->Whiten(A);

      //std::cout << "Whiten with mix idx " << idx << " A(0,0)=" << C(0,0) << std::endl;
      return C; 
    }

    void MaxMix::WhitenSystem(std::vector<gtsam::Matrix>& A, gtsam::Vector& b) const {
      ComponentSelect idx_gamma;

      b *= -1.0;
      b = this->processMixture(b, idx_gamma);  // eqivalent to whiten, but returning additional information for us
      b *= -1.0;

      for(Matrix& Aj: A) { 
        Aj = this->Whiten(Aj, idx_gamma.first); 
      }
    }

    void MaxMix::WhitenSystem(gtsam::Matrix& A, gtsam::Vector& b) const {
      ComponentSelect idx_gamma;
      b *= -1.0;
      b = this->processMixture(b, idx_gamma);  // eqivalent to whiten, but returning additional information for us
      b *= -1.0;
      A = this->Whiten(A, idx_gamma.first);
    }

    void MaxMix::WhitenSystem(gtsam::Matrix& A1, gtsam::Matrix& A2, gtsam::Vector& b) const {
      ComponentSelect idx_gamma;
      b *= -1.0;
      b = this->processMixture(b, idx_gamma);  // eqivalent to whiten, but returning additional information for us
      b *= -1.0;
      A1 = this->Whiten(A1, idx_gamma.first);
      A2 = this->Whiten(A2, idx_gamma.first);
    }

    void MaxMix::WhitenSystem(gtsam::Matrix& A1, gtsam::Matrix& A2, gtsam::Matrix& A3, gtsam::Vector& b) const{
      ComponentSelect idx_gamma;
      b *= -1.0;
      b = this->processMixture(b, idx_gamma);  // eqivalent to whiten, but returning additional information for us
      b *= -1.0;
      A1 = this->Whiten(A1, idx_gamma.first);
      A2 = this->Whiten(A2, idx_gamma.first);
      A3 = this->Whiten(A3, idx_gamma.first);
    }

    /* ****************************************************************************** */
    /* MaxSumMix */

    MaxSumMix::shared_ptr MaxSumMix::Create(const libmix4sam::Mixture& noise){
      return shared_ptr(new MaxSumMix(noise));
    }

    double MaxSumMix::squaredMahalanobisDistance(const Vector& v) const { 

      // normalization factor for all components
      gtsam::Vector c = this->mixture_.getScalingFactors();
      // whitened error for all components of mixture       
      gtsam::Matrix we = this->mixture_.getWhitenedComponents(v);
      
      c = c / c.maxCoeff(); // Modify scaling factor
      //c = c / c.sum(); // Modify scaling factor

      // Modify the normalization factors to be used as extra dimension in the error vector
      gtsam::Vector c_as_dim = (-2.0 * c.array().log()).sqrt();
      // Get index of component with minimum error
      size_t min_idx = this->min(c_as_dim, we);

      // Calculate correction term
      Vector exponents = this->mixture_.getExponents(v);
      double cMax = c(min_idx);
      double dMax = exponents(min_idx);
      double sumMixCorrection = -2.0 * (libmix4sam::ScaledLogSumExp(exponents.array() - dMax, c.array() / cMax) - log(c.size() + MAXSUMMIX_DAMPENING) );
      return this->cdistance(c_as_dim(min_idx), we.row(min_idx)) + sumMixCorrection;
    }

    gtsam::Vector MaxSumMix::whiten(const Vector& v, boost::optional<size_t &> idx_max) const {

      // normalization factor for all components
      gtsam::Vector c = this->mixture_.getScalingFactors();
      // whitened error for all components of mixture       
      gtsam::Matrix we = this->mixture_.getWhitenedComponents(v);
      
      c = c / c.maxCoeff(); // Modify scaling factor
      //c = c / c.sum(); // Modify scaling factor

      // Modify the normalization factors to be used as extra dimension in the error vector
      gtsam::Vector c_as_dim = (-2.0 * c.array().log()).sqrt();
      // Get index of component with minimum error
      size_t min_idx = this->min(c_as_dim, we);

      gtsam::Vector whitenedErrorMax = we.row(min_idx);

      // Calculate correction term
      Vector exponents = this->mixture_.getExponents(v);
      double cMax = c(min_idx);
      double dMax = exponents(min_idx);
      double sumMixCorrection = sqrt(-2.0 * (libmix4sam::ScaledLogSumExp(exponents.array() - dMax, c.array() / cMax) - log(c.size() + MAXSUMMIX_DAMPENING) ));

      if (idx_max) (*idx_max) = min_idx;

      return (gtsam::Vector(whitenedErrorMax.size()+1) << whitenedErrorMax, sumMixCorrection).finished();
    };

    Matrix MaxSumMix::Whiten(const Matrix& A, const Vector& v, const size_t& idx_max) const {
      // A ... 
      // v ... -unwhitenedError()

      // A corresponds to d_unwhitenedError/d_state
      //calculate: d_whiten/d_unwhitenedError
      Matrix d_whiten__d_unwhitenedError;

#if !defined(MAXSUMMIX_NUMERIC) || MAXSUMMIX_NUMERIC 
      if (v.size() == 1){
        d_whiten__d_unwhitenedError =
          numericalDerivative11<gtsam::Vector2, gtsam::Vector, 1>(
            boost::function<gtsam::Vector(const gtsam::Vector&)> (
              boost::bind( 
                &This::whiten,   /* Fehlerfunktion ist Methode einer Klasse */
                this,            /* Wir wollen die Methode einer Instanz der Klasse nutzen! */
                _1,boost::none)),            /* Platzhalter für 1. Übergabeparameter */
            v);                  /* 1. Übergabeparameter */
      }else{ // Problem: Numerical derivation needs fixed dimension, but dimension is not known at compile time!
        d_whiten__d_unwhitenedError =
          numericalDerivative11<gtsam::Vector3, gtsam::Vector, 2>(
            boost::function<gtsam::Vector(const gtsam::Vector&)> (
              boost::bind( 
                &This::whiten,   /* Fehlerfunktion ist Methode einer Klasse */
                this,            /* Wir wollen die Methode einer Instanz der Klasse nutzen! */
                _1,boost::none)),            /* Platzhalter für 1. Übergabeparameter */
            v);                  /* 1. Übergabeparameter */
      }
      return d_whiten__d_unwhitenedError * A;  // TODO: A hat höhere Dimension, da Jakobi-Matrix aus Faktor.
#else
      // As we extended our error vector, we need to add an additional line to the A matrix!
      gtsam::Matrix C = Matrix::Zero(A.rows()+1, A.cols());

      // Get the current mixture component
      const gtsam::noiseModel::Gaussian* p_max = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*mixture_.at(idx_max).noiseModel_);
      if (p_max == NULL) throw std::invalid_argument( "MaxSumMix::Whiten: can not convert one of the mixture component to a gaussian!" );

      // Whiten matrix by current mixture component
      C.topRows(A.rows()) = p_max->Whiten(A);

      // See technical documentation for details on the formulas!
      Vector exponents = this->mixture_.getExponents(v);
      gtsam::Vector c = this->mixture_.getScalingFactors();
      exponents = exponents.array() - exponents(idx_max); // \tilde d_{i,j}
      c = c.array() / (c(idx_max) * (c.size() + MAXSUMMIX_DAMPENING)); // \tilde c_j = \frac{c_j}{\gamma_l c_{max}}

      std::vector<gtsam::Vector> d_exponents__d_unwhitenedError;
      gtsam::Vector d_imax__d_unwhitenedError = p_max->information() * (v - mixture_.at(idx_max).mu_);
      for (size_t j=0; j<exponents.size();j++){
        // Currently a Gaussian based error model for each mixture component is needed, because of the needed square root information matrix
        const gtsam::noiseModel::Gaussian* p = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*this->mixture_.at(j).noiseModel_);
        if (p == NULL) throw std::invalid_argument( "SumMix: only gaussian based noise models are currently supported!");

        gtsam::Vector d_ij__d_unwhitenedError = p->information() * (this->mixture_.at(j).mu_ - v); // Vertauschen von mu und error ist korrekt!
        d_exponents__d_unwhitenedError.push_back(d_ij__d_unwhitenedError + d_imax__d_unwhitenedError);
      }

      std::vector<double> d_sLSE__d_exponents;
      double d_sLSE_denominator = (c.array() * exponents.array().exp()).sum();
      for (size_t j=0; j<exponents.size();j++){
        double d_sLSE_numerator = c(j) * exp(exponents(j));
        d_sLSE__d_exponents.push_back(d_sLSE_numerator / d_sLSE_denominator);
      }

      gtsam::Vector dSum = gtsam::Vector::Zero(v.size());
      for (size_t j=0; j<exponents.size();j++){
        dSum += d_sLSE__d_exponents.at(j) * d_exponents__d_unwhitenedError.at(j);
      }

      d_whiten__d_unwhitenedError = -1 / sqrt(-2*libmix4sam::ScaledLogSumExp(exponents, c)) * dSum.matrix().transpose();
      C.bottomRows(1) = d_whiten__d_unwhitenedError * A;

      return C;
#endif  
    };

    void MaxSumMix::WhitenSystem(std::vector<gtsam::Matrix>& A, gtsam::Vector& b) const {
      size_t idx_max;
      gtsam::Vector bw = this->whiten(-b, idx_max);
      for(Matrix& Aj: A)  Aj = this->Whiten(Aj,-b, idx_max);
      b = -bw;
    }

    void MaxSumMix::WhitenSystem(gtsam::Matrix& A, gtsam::Vector& b) const {
      size_t idx_max;
      gtsam::Vector bw = this->whiten(-b, idx_max);
      A = this->Whiten(A, -b, idx_max);
      b = -bw;
    }

    void MaxSumMix::WhitenSystem(gtsam::Matrix& A1, gtsam::Matrix& A2, gtsam::Vector& b) const {
      size_t idx_max;
      gtsam::Vector bw = this->whiten(-b, idx_max);
      A1 = this->Whiten(A1, -b, idx_max); A2 = this->Whiten(A2, -b, idx_max);
      b = -bw;
    }

    void MaxSumMix::WhitenSystem(gtsam::Matrix& A1, gtsam::Matrix& A2, gtsam::Matrix& A3, gtsam::Vector& b) const{
      size_t idx_max;
      gtsam::Vector bw = this->whiten(-b, idx_max);
      A1 = this->Whiten(A1, -b, idx_max); A2 = this->Whiten(A2, -b, idx_max);A3 = this->Whiten(A3, -b, idx_max);
      b = -bw;
    }

    /* ****************************************************************************** */
    /* SumMix */

    SumMix::shared_ptr SumMix::Create(const libmix4sam::Mixture& noise){
        return shared_ptr(new SumMix(noise));
    }

    bool SumMix::equals(const Base& expected, double tol) const {
        const SumMix* p = dynamic_cast<const SumMix*> (&expected);
        if (p == NULL) return false;
        return mixture_.equals(p->mixture(),tol);
    }

    void SumMix::print(const std::string& name) const {
        mixture_.print(name);
    }

    Vector SumMix::getScalings(boost::optional<double &> gamma_s) const {

      gtsam::Vector c = this->mixture_.getScalingFactors();

      double gamma_s_ = c.array().sum();

      // Optionally return the used gamma_s
      if (gamma_s) (*gamma_s) = gamma_s_;

      return c / gamma_s_;
    }

    Vector SumMix::processMixture(const Vector& v, double &gamma_s) const {
      return (this->getScalings(gamma_s).array() * this->mixture_.getExponents(v).array().exp()).matrix();
    }

    double SumMix::processMixtureNumericalRobust(const Vector& v, double &gamma_s) const {
        // return the gmm's log-likeliehood
        return libmix4sam::ScaledLogSumExp(this->mixture_.getExponents(v), this->getScalings(gamma_s));
    }

    Vector SumMix::whiten(const Vector& v) const {
      double gamma_s = 0;
      double logSumMixture = processMixtureNumericalRobust(v, gamma_s);
      //double sqrtLogSumMixture = sqrt(-2.0*(logSumMixture - log(gamma_s)));
      double sqrtLogSumMixture = sqrt(-2.0*logSumMixture);
      gtsam::Vector v_new = gtsam::Vector::Zero(1);
      v_new(0) = sqrtLogSumMixture;// * sqrt(2);
      return v_new;
    };

    Matrix SumMix::Whiten(const Matrix& A, const Vector& v) const {
      // A ... 
      // v ... -unwhitenedError()

      // A corresponds to d_unwhitenedError/d_state
      //calculate: d_whiten/d_unwhitenedError
      Matrix d_whiten__d_unwhitenedError;

#if !defined(SUMMIX_NUMERIC) || SUMMIX_NUMERIC 
      if (v.size() == 1){
        d_whiten__d_unwhitenedError =
          numericalDerivative11<gtsam::Vector1, gtsam::Vector, 1>(
            boost::function<gtsam::Vector(const gtsam::Vector&)> (
              boost::bind( 
                &This::whiten,   /* Fehlerfunktion ist Methode einer Klasse */
                this,            /* Wir wollen die Methode einer Instanz der Klasse nutzen! */
                _1)),            /* Platzhalter für 1. Übergabeparameter */
            v);                  /* 1. Übergabeparameter */
      }else{ // Problem: Numerical derivation needs fixed dimension, but dimension is not known at compile time!
        d_whiten__d_unwhitenedError =
          numericalDerivative11<gtsam::Vector1, gtsam::Vector, 2>(
            boost::function<gtsam::Vector(const gtsam::Vector&)> (
              boost::bind( 
                &This::whiten,   /* Fehlerfunktion ist Methode einer Klasse */
                this,            /* Wir wollen die Methode einer Instanz der Klasse nutzen! */
                _1)),            /* Platzhalter für 1. Übergabeparameter */
            v);                  /* 1. Übergabeparameter */
      }
#else
      // See technical documentation for details on the formulas!
      Vector exponents = this->mixture_.getExponents(v);
      std::vector<gtsam::Vector> dExponents = this->mixture_.getDExponents(v);
      Vector scalings = this->getScalings();
      
      gtsam::Vector dSum = gtsam::Vector::Zero(v.size());
      for (size_t j=0; j<exponents.size();j++){
        double d_sLSE__d_exponent = scalings(j)*libmix4sam::ScaledDivSumExp(exponents(j),exponents,scalings);
        gtsam::Vector d_exponent__d_unwhitenedError = dExponents.at(j);
        dSum += d_sLSE__d_exponent * d_exponent__d_unwhitenedError; // Note: in the end it is not a vector, but a matrix with colums corresponding to the dimension of the error vector
      }
      d_whiten__d_unwhitenedError = -1 / sqrt(-2*libmix4sam::ScaledLogSumExp(exponents, scalings)) * dSum.matrix().transpose();

#endif
      return d_whiten__d_unwhitenedError * A;  
    };

    void SumMix::WhitenSystem(std::vector<gtsam::Matrix>& A, gtsam::Vector& b) const {
      b *= -1.0;
      //std::cout << "SumMix WhitenSystem Vector(A) b_old=" << b;
      for(Matrix& Aj: A) { 
        //std::cout << " A_old=" << Aj(0,0);
        Aj = this->Whiten(Aj,b); 
        //std::cout << " A_new=" << Aj(0,0);
      }
      b = this->whiten(b);
      //std::cout << " b_new=" << b;
      //std::cout << std::endl;
      b *= -1.0;
    }

    void SumMix::WhitenSystem(gtsam::Matrix& A, gtsam::Vector& b) const {
      b *= -1.0;
      b = this->whiten(b);
      b *= -1.0;
      A = this->Whiten(A,b);
    }

    void SumMix::WhitenSystem(gtsam::Matrix& A1, gtsam::Matrix& A2, gtsam::Vector& b) const {
      //std::cout << "SumMix WhitenSystem A1 A2 b" << std::endl;
      
      b *= -1.0;
      b = this->whiten(b);
      b *= -1.0;
      A1 = this->Whiten(A1,b);
      A2 = this->Whiten(A2,b);
    }

    void SumMix::WhitenSystem(gtsam::Matrix& A1, gtsam::Matrix& A2, gtsam::Matrix& A3, gtsam::Vector& b) const{
      //std::cout << "SumMix WhitenSystem A1 A2 A3 b" << std::endl;
      
      b *= -1.0;
      b = this->whiten(b);
      b *= -1.0;
      A1 = this->Whiten(A1,b);
      A2 = this->Whiten(A2,b);
      A3 = this->Whiten(A3,b);
    }

  }

}


