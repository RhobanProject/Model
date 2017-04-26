#include <cmath>
#include <stdexcept>
#include "Utils/GaussianDistribution.hpp"

namespace Leph {

GaussianDistribution::GaussianDistribution() :
    _mean(),
    _covariance(),
    _choleskyDecomposition(),
    _determinant(0.0)
{
}

GaussianDistribution::GaussianDistribution(
    const Eigen::VectorXd& mean,
    const Eigen::MatrixXd& covariance) :
    _mean(mean),
    _covariance(covariance),
    _choleskyDecomposition(),
    _determinant(0.0)
{
    //Check size
    if (
        mean.size() != covariance.rows() ||
        mean.size() != covariance.cols()
    ) {
        throw std::logic_error(
            "GaussianDistribution invalid input size");
    }
    //Cholesky decomposition of covariance matrix
    computeDecomposition();
}
        
size_t GaussianDistribution::dimension() const
{
    return _mean.size();
}

const Eigen::VectorXd& GaussianDistribution::mean() const
{
    return _mean;
}
const Eigen::MatrixXd& GaussianDistribution::covariance() const
{
    return _covariance;
}

Eigen::VectorXd GaussianDistribution::sample(
    std::default_random_engine& engine) const
{
    Eigen::VectorXd unitRand(_mean.size());
    std::normal_distribution<double> dist(0.0, 1.0);
    for (size_t i=0;i<(size_t)unitRand.size();i++) {
        unitRand(i) = dist(engine);
    }
    return _mean + _choleskyDecomposition*unitRand;
}

double GaussianDistribution::probability(
    const Eigen::VectorXd& point) const
{
    size_t size = _mean.size();
    if ((size_t)point.size() != size) {
        throw std::logic_error(
            "GaussianDistribution invalid dimension");
    }

    Eigen::VectorXd delta = point - _mean;
    double tmp1 = -0.5*delta.transpose()*_covarianceInv*delta;
    double tmp2 = pow(2.0*M_PI, size)*_determinant;
    return std::exp(tmp1)/std::sqrt(tmp2);
}

double GaussianDistribution::logProbability(
    const Eigen::VectorXd& point) const
{
    size_t size = _mean.size();
    if ((size_t)point.size() != size) {
        throw std::logic_error(
            "GaussianDistribution invalid dimension");
    }

    Eigen::VectorXd delta = point - _mean;
    double tmp1 = delta.transpose()*_covarianceInv*delta;
    return -0.5*(
        std::log(_determinant) 
        + tmp1 
        + (double)size*std::log(2.0*M_PI));
}

void GaussianDistribution::fit(
    const std::vector<Eigen::VectorXd>& data)
{
    //Check sizes
    if (data.size() < 2) {
        throw std::logic_error(
            "GaussianDistribution not enough data points");
    }
    size_t size = data.front().size();
    for (size_t i=0;i<data.size();i++) {
        if ((size_t)data[i].size() != size) {
            throw std::logic_error(
                "GaussianDistribution invalid data dimension");
        }
    }
    
    //Compute the mean estimation
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(size);
    for (size_t i=0;i<data.size();i++) {
        sum += data[i];
    }
    _mean = (1.0/(double)data.size())*sum;
    
    //Compute the covariance estimation
    Eigen::MatrixXd sum2 = Eigen::MatrixXd::Zero(size, size);
    for (size_t i=0;i<data.size();i++) {
        sum2 += (data[i]-_mean)*(data[i]-_mean).transpose();
    }
    _covariance = (1.0/(double)(data.size()-1))*sum2;
    //Update the Cholesky decomposition
    computeDecomposition();
}
        
void GaussianDistribution::computeDecomposition()
{
    //Add small epsilon on diagonal according 
    //to Rasmussen 2006
    double epsilon = 1e-9;
    size_t size = _mean.size();
    Eigen::LLT<Eigen::MatrixXd> llt(
        _covariance 
        + epsilon*Eigen::MatrixXd::Identity(size, size));
    _choleskyDecomposition = llt.matrixL();
    //Check the decomposition
    if(llt.info() == Eigen::NumericalIssue) {
        throw std::logic_error(
            "GaussianDistribution Cholesky decomposition error");
    }
    //Compute the covariance determinant
    _determinant = 1.0;
    for (size_t i=0;i<(size_t)_mean.size();i++) {
        _determinant *= _choleskyDecomposition(i, i);
    }
    _determinant = pow(_determinant, 2);
    //Compute the covariance inverse
    _covarianceInv = llt.solve(
        Eigen::MatrixXd::Identity(size, size));
}

}

