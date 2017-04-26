#ifndef LEPH_GAUSSIANDISTRIBUTION_HPP
#define LEPH_GAUSSIANDISTRIBUTION_HPP

#include <vector>
#include <random>
#include <Eigen/Dense>

namespace Leph {

/**
 * GaussianDistribution
 *
 * Multivariate gaussian 
 * normal distribution
 */
class GaussianDistribution
{
    public:

        /**
         * Dummy empty initialization
         */
        GaussianDistribution();

        /**
         * Initialization with 
         * mean vector and covariance 
         * matrix
         */
        GaussianDistribution(
            const Eigen::VectorXd& mean,
            const Eigen::MatrixXd& covariance);

        /**
         * Return the gaussian 
         * dimentionality
         */
        size_t dimension() const;

        /**
         * Access to internal mean vector
         * and covariance matrix
         */
        const Eigen::VectorXd& mean() const;
        const Eigen::MatrixXd& covariance() const;

        /**
         * Sample a vector from the
         * multivariate gaussian with 
         * given random engine
         */
        Eigen::VectorXd sample(
            std::default_random_engine& engine) const;

        /**
         * Return the probability of given
         * vector point and current normal law
         */
        double probability(
            const Eigen::VectorXd& point) const;

        /**
         * Return the log-probability of given
         * vector point and current normal law
         */
        double logProbability(
            const Eigen::VectorXd& point) const;

        /**
         * Compute the classic estimation of gaussian
         * mean and covariance from given data vectors
         */
        void fit(
            const std::vector<Eigen::VectorXd>& data);

    private:

        /**
         * The mean vector
         */
        Eigen::VectorXd _mean;

        /**
         * The covariance matrix
         * (symetrix definite positive)
         */
        Eigen::MatrixXd _covariance;

        /**
         * The inverse of covariance matrix computed 
         * through cholesky decomposition
         */
        Eigen::MatrixXd _covarianceInv;

        /**
         * The left side of the Cholesky
         * decomposition of the covariance matrix
         */
        Eigen::MatrixXd _choleskyDecomposition;

        /**
         * The determinant of the 
         * covariance matrix
         */
        double _determinant;

        /**
         * Compute the covariance decomposition
         */
        void computeDecomposition();
};

}

#endif

