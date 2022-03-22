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
 * @file NoiseModelNew.h
 * @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
 * @brief Implements robust noise models.
 */

#ifndef NOISEMODELNEW_H
#define NOISEMODELNEW_H

#include <iostream>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtsam/base/timing.h>
//#define gttic_(label) ((void)0)

using namespace gtsam;

namespace libmix4sam
{

  struct MixComponent {

    gtsam::noiseModel::Base::shared_ptr noiseModel_;   /**< Component's noise model (currently only Gaussians are supported) */
    double w_;                                         /**< Weight of component */
    gtsam::Vector mu_;                                 /**< Mean value of component */

    MixComponent(const gtsam::noiseModel::Base::shared_ptr noiseModel, double w, gtsam::Vector mu);
    MixComponent(const gtsam::noiseModel::Base::shared_ptr noiseModel, double w) :
        noiseModel_(noiseModel), w_(w) { mu_ = gtsam::Vector::Zero(noiseModel->dim());};

    /**< matlab wrapper requires a destructor */
    ~MixComponent() {}

    /**
     * @brief Distance metric based on G. Sfikas et.al 2005.
     * See: "An Analytic Distance Metric for Gaussian Mixture Models with Application in Image Retrieval"
     * (G.Sfikas, C.Constantinopoulos, A.Likas and N.P.Galatsanos)
     * 
     * @param other 
     * @return double 
     */
    double sfikas05(const MixComponent &other) const{
      // Ungeeignet! Vergleicht komplette Mixtures miteinander.
      return 0.0;
    }

    /// Check for changes in model
    double md5() const{
      const gtsam::noiseModel::Gaussian* n1 = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*this->noiseModel_);
      return this->mu_.array().sum() + this->w_ + n1->R().array().sum();
    }

    /**
     * @brief Calculate the Bhattacharyya distance relative to another mixture component.
     * 
     * @param other Mixture component to calculate the distance to.
     * @return double Scalar value for the distance.
     */
    double bhattacharyya(const MixComponent &other) const;

    // For Matlab wrapper
    gtsam::noiseModel::Base::shared_ptr noiseModel() const {return noiseModel_;}
    double w() const {return w_;}
    gtsam::Vector mu() const {return mu_;}
    void print(const std::string& name) const;

    inline bool operator==(const MixComponent& rhs) const {
      return (w_ == rhs.w_) && (mu_ == rhs.mu_) && noiseModel_->equals(*rhs.noiseModel_);
    }
  };


  /**
   * @brief Representation of a mixture model.
   *
   * (code inspired by "struct DiscreteKeys")
   * This should only be a storage class, without additional functionalities!
   */
  class Mixture : public std::vector<MixComponent> {
    protected:

    size_t dim_; /**< Dimension of mixture model (e.g. dimension of mean value) */

    mutable gtsam::Vector cacheScalingFactors_;

    public:

    /// Default Constructor
    Mixture() : dim_(0) {}

    /**< matlab wrapper requires a destructor */
    ~Mixture() {}

    /// Construct from a component
    Mixture(const MixComponent& c);

    /// Construct from a vector of components
    Mixture(const std::vector<MixComponent>& components) : std::vector<MixComponent>(components) { dim_ = components.at(0).noiseModel_->dim(); /*TODO*/ }

    /// Add a key (non-const!)
    Mixture& operator&(const MixComponent& component);

    /// Convenience Method with relevance to Matlab-Wrapper
    void add(const MixComponent& c);

    /**
     * @brief Add all components from other mixture distribution. 
     * This normalizes the weights first and uses the influence percentage to weight the components of the two mixtures accordingly.
     * 
     * @param other 
     * @param influence 
     */
    void add(const Mixture& other, double influence);

    /**
     * @brief Temporary to add additional covariance to all gaussian components
     * Temporary, because a correct distribution to distribution implementation for RadarFactor is not easy!
     */
    void addToAll(const gtsam::SharedNoiseModel& g) {
      const gtsam::noiseModel::Gaussian* p = dynamic_cast<const gtsam::noiseModel::Gaussian*> (&*g);
      if (p == NULL) {std::cout << "Mixture::addToAll: can not convert the given noise model to a gaussian!" << std::endl; return;}
      addToAll(p->covariance());
    };

    void addToAll(const gtsam::Matrix& covar){
      for (size_t i=0;i<this->size();i++){
        gtsam::noiseModel::Gaussian* p2 = dynamic_cast<gtsam::noiseModel::Gaussian*> (&*this->at(i).noiseModel_);
        if (p2 == NULL) {std::cout << "Mixture::addToAll: can not convert one of the mixture component to a gaussian!" << std::endl; return;}
        this->at(i).noiseModel_ = gtsam::noiseModel::Gaussian::Covariance(covar + p2->covariance());
      }
      this->cacheScalingFactors_ = gtsam::Vector(); // reset scaling factors cache
    };

    /**
     * @brief Calculates the normalization factor c_j for each component.
     *
     * \f[ c_{j}=w_{j} \cdot \mathrm{det}\left(\mathcal{I}_{j}^{\frac{1}{2}}\right) \f]
     * with \f$w_j\f$ the weight and \f$\mathcal{I}_{j}^{\frac{1}{2}}\f$ the square root information matrix of component \f$ j\f$
     * 
     * @note Even if const method, the calculated scaling factors are cached within a mutable member variable.
     * 
     * @return c_j
     */
    gtsam::Vector getScalingFactors() const;

    /**
     * @brief Returns for all mixture components the whitened error vector.
     * The error vector for the i-th component corresponds to the i-th row of the matrix we.
     * 
     * \f[ \boldsymbol{\mathcal { I }}_{j}^{\frac{1}{2}}\left(\mathbf{e}_{i}-\boldsymbol{\mu}_{j}\right) \f]
     * With \f$\boldsymbol{\mu}_{j}\f$ beeing the mean vector and \f$\mathcal{I}_{j}^{\frac{1}{2}}\f$ the square root information matrix of component \f$ j\f$.
     *
     * @param[in] v unwhitened error vector.
     * @return Matrix with whitened error vectors.
     */
    gtsam::Matrix getWhitenedComponents(const gtsam::Vector& v) const;

    /**
     * @brief Calculate the Exponents for each component's gaussian distribution.
     *
     * \f[ -\frac{1}{2}\left\|\boldsymbol{\mathcal { I }}_{j}^{\frac{1}{2}}\left(\mathbf{e}_{i}-\boldsymbol{\mu}_{j}\right)\right\|^{2} \f]
     * With \f$\boldsymbol{\mu}_{j}\f$ beeing the mean vector and \f$\mathcal{I}_{j}^{\frac{1}{2}}\f$ the square root information matrix of component \f$ j\f$.
     * 
     * @param unwhitenedError Error vector.
     * @return Vector of exponents for each GMM component.
     */
    Vector getExponents(const gtsam::Vector &unwhitenedError) const;

    /**
     * @brief Calculate the derivatives of an exponent with respect to the unwhitenedError vector.
     *
     * @param unwhitenedError
     * @return std::vector<gtsam::Vector> Derivatives for each component of the mixture.
     */
    std::vector<gtsam::Vector> getDExponents(const gtsam::Vector &unwhitenedError) const;

    /// Return all components' weights as vector. 
    gtsam::Vector w() const;
    void setWeights(const gtsam::Vector &weights);
    void normalizeWeights();

    /// Check for changes in model
    double md5() const {
      double md5sum = 0;
      for (size_t i=0;i<this->size();i++) md5sum += this->at(i).md5();
      return md5sum;
    }

    /**
     * @brief Merge all components within the mixture to one.
     * (This only makes sense, if the components are close to each other.)
     *
     * @return MixComponent
     */
    MixComponent merge() const;

    size_t dim() const {return this->dim_;};
    bool equals(const Mixture& expected, double tol=1e-9) const;
    void print(const std::string& name) const;

  };

  /// Create a Mixture from two Mixture Components
  Mixture operator&(const MixComponent& c1, const MixComponent& c2);

  /**
   * @brief Class with multiple mixtures.
   * Use this class to simplify mixtures.
   */
  class MixtureClustered : public std::vector<Mixture> {

    public:

    /**< Use each component of a mixture distribution as single cluster. */
    MixtureClustered(const Mixture& mix){
      for(size_t i=0; i<mix.size(); i++)
        push_back(Mixture(mix.at(i)));
    }

    /**< matlab wrapper requires a destructor */
    ~MixtureClustered() {}

    void print(const std::string& name) const;

    /**
     * @brief Calculates the Bhattacharyya distance for each cluster pair.
     * Method returns the distance for each cluster combination within a vector
     * accompanied by a two column matrix with the first and second cluster index
     * in each row.
     * 
     * @return std::pair<gtsam::Vector, gtsam::Matrix> 
     */
    std::pair<gtsam::Vector, gtsam::Matrix> distBhattacharyya() const;

    /**
     * @brief Merge all components which are similar.
     * 
     * @param threshold Threshold for merging two components.
     * @return Bool 
     */
    bool simplify(double threshold = 0.2);

    /**
     * @brief Convert the clusters back into a mixture distribution.
     * 
     * @return Mixture Returns the mixture distribution described by the clusters.
     */
    Mixture getAsMixture() const;

    /**
     * @brief Merge two clusters. 
     * E.g. if cluster 1 has two components and cluster 2 as one, the resulting cluster will be a mixture with 3 components.
     *
     * @param index1 Index of cluster 1.
     * @param index2 Index of cluster 2.
     */
    void merge(size_type index1, size_type index2);

  };

  namespace noiseModelNew
  {

    /**
     * @brief Base class for implementing mixture noise models.
     *
     */
    class MixBase : public gtsam::noiseModel::Base
    {

      protected:
        libmix4sam::Mixture mixture_;    ///< Mixture of noiseModels

        /// Default Constructor for serialization
        MixBase() {};

        /// primary constructor @param dim is the dimension of the model
        MixBase(size_t dim) : Base(dim) {};

        /// Constructor
        MixBase(const libmix4sam::Mixture& mix) : Base(mix.dim()), mixture_(mix) {};

        virtual ~MixBase() {}

      public:

        typedef boost::shared_ptr<MixBase> shared_ptr;

        virtual shared_ptr cloneWithNewMix(const libmix4sam::Mixture& mix) const = 0;

        /// Return the contained Mixture model
        const Mixture& mixture() const { return mixture_; }

        virtual void print(const std::string& name) const;

        virtual Vector whiten(const Vector& v) const;

        inline virtual Vector unwhiten(const Vector& /*v*/) const
        { throw std::invalid_argument("unwhiten is not currently supported for MixBase noise models."); }

        /**
         * @brief For matlab interface to test the Whiten functionality.
         *
         * @param A
         * @param v
         * @return Matrix
         */
        virtual Matrix Whiten(const Matrix& A, const Vector& v) const = 0;

        // Attention. WhitenSystem methods use the vector b, which is typically corresponding to -1 * unwhitenedError
        // virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const;
        // virtual void WhitenSystem(Matrix& A, Vector& b) const;
        // virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const;
        // virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const;

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
          ar & BOOST_SERIALIZATION_NVP(mixture_);
        }
    };

    /**
     * @brief Implementation of a Maximum Mixture Noise Model following GtSAM conventions (hopefully).
     *
     */
    class MaxMix : public MixBase
    {
      public:
        typedef boost::shared_ptr<libmix4sam::noiseModelNew::MaxMix> shared_ptr;

        /**
         * @brief Type for additional information about the used / selected mixture component.
         * first ... Selected Component's index of Mixture distribution
         * second ... normalization factor gamma_m
         */
        typedef std::pair<size_t,double> ComponentSelect;

      protected:
        typedef gtsam::noiseModel::Gaussian NoiseModel;

        /// Default Constructor for serialization
        MaxMix() {};

        /// Constructor
        MaxMix(const libmix4sam::Mixture& mix) : MixBase(mix) {}

        mutable gtsam::Vector cacheErrorVector_; // last used error vector
        mutable double cacheMixSum_;  // Checksum for mixture distribution
        mutable gtsam::Vector cacheWhitenedError_;
        mutable size_t cacheNormalizerIdx_;
        mutable double cacheNormalizerValue_;


      public:

        /// Destructor
        virtual ~MaxMix() {}

        MixBase::shared_ptr cloneWithNewMix(const libmix4sam::Mixture& mix) const {
          return MixBase::shared_ptr(new MaxMix(mix));
        }

        virtual void print(const std::string& name) const;
        virtual bool equals(const Base& expected, double tol=1e-9) const;

        /**
         * @brief ONLY For Matlab interface to get the current whitened error in combination with the regularizer.
         *
         * @return std::pair<double,Vector> first is cj, normalization factor of gmm component (wj*det(R)) and second is we, whitened error of vector v
         */
        std::pair<double,Vector> regularWhite(const Vector& v) const{
          // see process mixture for more comments
          gtsam::Vector c = this->mixture_.getScalingFactors();
          gtsam::Matrix we = this->mixture_.getWhitenedComponents(v);
          gtsam::Vector c_dim = c;
          this->normalizer2dim(c_dim);
          size_t min_idx = this->min(c_dim, we);
          return std::make_pair(c(min_idx), we.row(min_idx));
        }

        /**
         * @brief Modify the gmm normalization term to be equal to an additional dimension.
         * see Pfeifer. 2019. "Expectation-Maximization for Adaptive Mixture Models in Graph Optimization", Eqn. 9
         *
         * @param c Vector of normalization factors.
         */
        void normalizer2dim(Vector &c) const;

        /**
         * @brief Search for the index of gaussian component with minimum error.
         *
         * @param c
         * @param we
         * @return size_t Index j of the minimum component.
         */
        size_t min(Vector &c, Matrix &we) const;

        /**
         * @brief Get mixture component with minimum cost.
         * 
         * Calculates the normalization factor, see normalizations() and normalizer2dim(), 
         * and the whitened error, see whitenedComponents() for each component of the GMM 
         * for selecting the component with minimal cost.
         *
         * @note Even if method is const, it changes the mutable cache variables of the class.
         *
         * @param[in]  v       Unwhitened error vector.
         * @param[out] gamma_m Returns index of selected component and the value of the additional normalizer dimension see normalizer2dim().
         * @return Vector Whitened error of the mixture's component with minimum cost.
         */
        Vector processMixture(const Vector& v, boost::optional<ComponentSelect &> gamma_m = boost::none) const;

        /// Standard interface can not be used, because of multiple components, overwrite for safety reasons.
        virtual Matrix Whiten(const Matrix& A) const {
          std::cout << "MaxMix, Whiten(A): DONT CALL THIS! USE Whiten(A,b)" << std::endl;
          return Matrix::Zero(0,0);
        };

        /// Whiten A with specific component of GMM, selected by given component's index.
        virtual Matrix Whiten(const Matrix& A, const size_t& idx) const;

        /// Same as WhitenSystem(), but only returning the whitened Jacobian matrix.
        virtual Matrix Whiten(const Matrix& A, const Vector& v) const {
          ComponentSelect idx_gamma;
          this->processMixture(v, idx_gamma);
          return this->Whiten(A, idx_gamma.first);
        };

        /**
         * @brief Uses processMixture() for searching and selecting the max component.
         * 
         * Using the selected component, their square root information matrix will be multiplied to A and b.
         * 
         * @note WhitenSystem methods use the vector b, which is typically corresponding to -1 * unwhitenedError
         *
         * @param[in,out] A Unwhitened Jacobian matrix to be converted into the A matrix.
         * @param[in,out] b Negative unwhitened error vector.
         */
        virtual void WhitenSystem(Matrix& A, Vector& b) const;
        
        // Convenience methods, doing the same as above with different input.
        virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const;
        virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const;
        virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const;

        /**
         * @brief Return the squared norm for one component.
         * 
         * \f[ \texttt{cdistance\{cj,vj\} } = \left\|\begin{array}{c} {c_{j}} \\ {\mathbf{v}_{j}} \end{array}\right\|^{2} \f]
         *
         * @param cj Normalization factor \f$\texttt{cj} = \sqrt{-2 \ln \frac{c_{j}}{\gamma_{m}}}\f$.
         * @param vj Whitened Error vector.
         * @return double
         */
        double cdistance(const double& cj, const Vector& vj) const {
          return cj*cj + vj.squaredNorm();
        }

        /**
         * @brief Calculates the squared Error of the factor.
         * The basic implementation of NoiseModelFactor::error uses this method to get the squared error.
         * The result will be multiplied by 0.5 (comming from loss()-method) within the NoiseModelFactor::error implementation.
         * Basically, this is the whitened error including an additional constant regularization term.
         *
         * @param v unwhitened error vector comming from the NonlinearFactor implementation.
         * @return double squared error including the regularization therm.
         */
        inline virtual double squaredMahalanobisDistance(const Vector& v) const { 
          ComponentSelect gamma_m;
          Vector d = this->processMixture(v, gamma_m);
          return this->cdistance(gamma_m.second, d);
        }

        /**
         * @brief Searches for the mixture component with minimum cost and uses its whitening method.
         * This means multiplying the sqare root information matrix to the error vector.
         *
         * @param v unwhitened error comming from the NonlinearFactor implementation.
         * @return Vector whitened error.
         */
        virtual Vector whiten(const Vector& v) const {
          // Hier wird der Fehlervektor mit der sqare root information matrix multipliziert.
          return processMixture(v);
          // TODO: Only if we want to optimize over the standard deviations or the weights of the components, we need to add the
          // extra dimension of the error vector and accordingly the extra entries in the A matrix

          // Add the regularizer as additional dimension
          //v = (gtsam::Vector(1 + v.rows()) << regularizer_, v).finished();
        }

        /// For Matlab to compensate for the distance normalizer.
        double distanceNormalizer(const Vector& v) const {
          ComponentSelect gamma_m;
          this->processMixture(v, gamma_m);
          return gamma_m.second; 
        }

        // Used in matlab interface
        static shared_ptr Create(const libmix4sam::Mixture& noise);

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MixBase);
        }
    };

    /**
     * @brief Implementation of a Maximum Mixture Noise Model combining Sum Mixture following GtSAM conventions (hopefully).
     *
     */
    class MaxSumMix : public MaxMix
    {
      public:
        typedef boost::shared_ptr<libmix4sam::noiseModelNew::MaxSumMix> shared_ptr;

      private:
        typedef MaxSumMix This;

      protected:

        /// Default Constructor for serialization
        MaxSumMix() {};

        /// Constructor
        MaxSumMix(const libmix4sam::Mixture& mix) : MaxMix(mix) {}

      public:

        /// Destructor
        virtual ~MaxSumMix() {}

        MixBase::shared_ptr cloneWithNewMix(const libmix4sam::Mixture& mix) const {
          return MixBase::shared_ptr(new MaxSumMix(mix));
        }
        
        virtual double squaredMahalanobisDistance(const Vector& v) const;

        virtual Vector whiten(const Vector& v, boost::optional<size_t &> idx_max = boost::none) const;
        Matrix Whiten(const Matrix& A, const Vector& v, const size_t& idx_max) const;

        virtual Matrix Whiten(const Matrix& A) const {std::cout << "SumMix::Whiten(A) -> NOT IMPLEMENTED. SHOULD NOT BE CALLED." << std::endl; return Matrix::Zero(0,0);};
        virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const ;
        virtual void WhitenSystem(Matrix& A, Vector& b) const ;
        virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const ;
        virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const ;

        // Used in matlab interface
        static shared_ptr Create(const libmix4sam::Mixture& noise);

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MaxMix);
        }
    };

    /**
     * @brief Implementation of a Maximum Mixture Noise Model following GtSAM conventions (hopefully).
     *
     */
    class SumMix : public MixBase
    {
      public:
        typedef boost::shared_ptr<libmix4sam::noiseModelNew::SumMix> shared_ptr;

      private:
        typedef SumMix This;

      protected:

        /// Default Constructor for serialization
        SumMix() {};

        /// Constructor
        SumMix(const libmix4sam::Mixture& mix) : MixBase(mix) {}

      public:

        /// Destructor
        virtual ~SumMix() {}

        MixBase::shared_ptr cloneWithNewMix(const libmix4sam::Mixture& mix) const {
          return MixBase::shared_ptr(new SumMix(mix));
        }

        virtual void print(const std::string& name) const;
        virtual bool equals(const Base& expected, double tol=1e-9) const;

        // gamma_s is for compensating the additionally used gamma_s within the whitenedError for the distance method

        /**
         * @brief Calculate the scaling for each component (c_j / gamma_s)
         *
         * @param gamma_s Optinally returns the value of used gamma_s
         * @return Vector Scaling for each component as vector entry
         */
        Vector getScalings(boost::optional<double &> gamma_s = boost::none) const;

        /**
         * @brief Calculate likeliehood for each gmm-component.
         * The following equation will be evaluated:
         * \f$\frac{c_{j}}{\gamma_{s}} \cdot \exp \left(-\frac{1}{2}\left\|\mathcal{I}_{j}^{\frac{1}{2}}\left(\mathbf{e}_{i}-\boldsymbol{\mu}_{j}\right)\right\|^{2}\right)\f$
         * where
         * \f$c_{j}=w_{j} \cdot \operatorname{det}\left(\mathcal{I}_{j}^{\frac{1}{2}}\right)\f$
         *
         * @param v
         * @param gamma_s Additional parameter, which returns the calculated value for gamma_s.
         * @return Vector Is returning the likeliehood for each component as a vector.
         */
        Vector processMixture(const Vector& v,  double &gamma_s) const;

        /**
         * @brief Calculate the GMM's log-likeliehood in a robust way.
         * (Based on code of Tim Pfeifer)
         *
         * @param[in] v
         * @param[out] gamma_s Additional parameter, which returns the calculated value for gamma_s.
         * @return double The GMM's likeliehood with already applied log.
         */
        double processMixtureNumericalRobust(const Vector& v, double &gamma_s) const;

        inline virtual double squaredMahalanobisDistance(const Vector& v) const  {
          // double gamma_s = 1.0;
          // Vector gm = this->processMixture(v, gamma_s);
          // // To mach normal conventions, multiply by two and compensate for the introduced normalization factor
          // return -2.0 * ( log(gamma_s) + log(gm.array().sum()));

          // Wir berechnen den whitened error mit hilfe numerisch stabiler Funktionen
          double gamma_s = 0;
          double logSumMixture = processMixtureNumericalRobust(v, gamma_s);
          return -2.0*logSumMixture;

        }

        /**
         * @brief Uses the mixture component choosen by the previous run of whiten and use this
         *        component for whitening the A matrix.
         * This means multiplying the square root information matrix to A.
         * ATTENTION: whiten has always to be run first, to have the correct mixture component.
         *
         * @param A unwhitened Jacobian matrix to be converted into the A matrix.
         * @return Matrix whitened Jacobian
         */
        virtual Matrix Whiten(const Matrix& A) const  {
          // Wir machen gar nichts, da wir im Fall von SumMix die Kovarianz bereits in evaluateError berücksichtigen müssen.
          // Das übliche Auftrennen in \Sigma^{-1/2}*A funktioniert hier nicht. Die Gewichtung für A muss bereits eingeflossen sein!
          //std::cout << "SumMix::Whiten = TODO" << std::endl;

          // A corresponds to d_unwhitenedError/d_state
          //calculate: d_whiten/d_unwhitenedError

          std::cout << "SumMix::Whiten(A) -> NOT IMPLEMENTED. SHOULD NOT BE CALLED." << std::endl;
          return Matrix::Zero(0,0);
        };

        Matrix Whiten(const Matrix& A, const Vector& v) const ;

        // Attention. WhitenSystem methods use the vector b, which is typically corresponding to -1 * unwhitenedError
        virtual void WhitenSystem(std::vector<Matrix>& A, Vector& b) const ;
        virtual void WhitenSystem(Matrix& A, Vector& b) const ;
        virtual void WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const ;
        virtual void WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const ;

        /**
         * @brief Uses processMixtureNumericalRobust to calculate the likeliehood of error vector.
         * Additionally, it takes the normalization constatnt gamma_s into account and takes the square root.
         *
         * @param v unwhitened error comming from the NonlinearFactor implementation.
         * @return Vector whitened error.
         */
        virtual Vector whiten(const Vector& v) const ;

        // Used in matlab interface
        static shared_ptr Create(const libmix4sam::Mixture& noise);

      private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class ARCHIVE>
        void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
          ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MixBase);
        }

    };


  }
}

#endif // NOISEMODELNEW_H
