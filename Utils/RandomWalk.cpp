#include <iostream>
#include <stdexcept>
#include "Utils/RandomWalk.hpp"

namespace Leph {

RandomWalk::RandomWalk(size_t dim) :
    _dim(dim),
    _statePos(Eigen::VectorXd::Zero(dim)),
    _stateVelNormalized(Eigen::VectorXd::Zero(dim)),
    _stateVel(Eigen::VectorXd::Zero(dim)),
    _generator(std::random_device{}())
{
}
RandomWalk::RandomWalk(const Eigen::VectorXd& initState) :
    _dim(initState.size()),
    _statePos(initState),
    _stateVelNormalized(Eigen::VectorXd::Zero(_dim)),
    _stateVel(Eigen::VectorXd::Zero(_dim)),
    _generator()
{
}
        
const Eigen::VectorXd& RandomWalk::step(
    double deltaMean, double inertiaRatio)
{
    if (inertiaRatio > 1.0 || inertiaRatio < 0.0) {
        throw std::logic_error("RandomWalk invalid inertiaRatio");
    }

    //Compute normalized velocity
    Eigen::VectorXd unit = randomUnit();
    _stateVelNormalized = 
        inertiaRatio*_stateVelNormalized 
        + (1.0-inertiaRatio)*unit;
    //Scale velocity to asked value
    _stateVel = deltaMean*_stateVelNormalized;
    //Position integration
    _statePos += _stateVel;

    return _statePos;
}

const Eigen::VectorXd& RandomWalk::statePos() const
{
    return _statePos;
}
Eigen::VectorXd& RandomWalk::statePos()
{
    return _statePos;
}
const Eigen::VectorXd& RandomWalk::stateVel() const
{
    return _stateVel;
}
Eigen::VectorXd& RandomWalk::stateVel()
{
    return _stateVel;
}
        
Eigen::VectorXd RandomWalk::randomUnit()
{
    Eigen::VectorXd unit(_dim);
    
    std::normal_distribution<double> normalRand(0.0, 1.25);
    for (size_t i=0;i<_dim;i++) {
        unit(i) = normalRand(_generator);
    }

    return unit;
}

}

