#include <Utils/Angle.h>
#include "Model/OdometryModel.hpp"

namespace Leph {
        
OdometryModel::OdometryModel() :
    _isInitialized(false),
    _odometryParameters(Eigen::VectorXd::Zero(8)),
    _support(),
    _last(),
    _state(),
    _corrected()
{
    //Default parameters
    _odometryParameters <<
        0.0, //dX = offset
        1.0, //dX = dx
        0.0, //dX = dy
        0.0, //dX = dtheta
        0.0, //dY = offset
        0.0, //dY = dx
        1.0, //dY = dy
        0.0; //dY = dtheta
    //Ask reset
    reset();
}
        
void OdometryModel::reset()
{
    _isInitialized = false;
    _state = Eigen::Vector3d(0.0, 0.0, 0.0);
    _corrected = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void OdometryModel::update(
    const Eigen::Vector3d& current, 
    Leph::HumanoidFixedModel::SupportFoot supportFoot)
{
    if (!_isInitialized) {
        _support = supportFoot;
        _last = current;
        _isInitialized = true;
    }

    //Retrieve current model state
    Leph::HumanoidFixedModel::SupportFoot lastSupport = _support;
    _support = supportFoot;
    
    //Compute displacement between current Model 
    //state and last support swap
    Eigen::Vector3d diff = odometryDiff(_last, current);
    //Apply displacement correction
    double diffX = 
        _odometryParameters(0) +
        _odometryParameters(1)*diff.x() +
        _odometryParameters(2)*diff.y() +
        _odometryParameters(3)*diff.z();
    double diffY = 
        _odometryParameters(4) +
        _odometryParameters(5)*diff.x() +
        _odometryParameters(6)*diff.y() +
        _odometryParameters(7)*diff.z();
    diff.x() = diffX;
    diff.y() = diffY;

    //Update corrected odometry at support swap
    if (
        lastSupport == Leph::HumanoidFixedModel::RightSupportFoot &&
        _support == Leph::HumanoidFixedModel::LeftSupportFoot
    ) {
        //Integrate the displacement at corrected odometry state
        odometryInt(diff, _state);
        //Save current Model state
        _last = current;
        //Update corrected state
        _corrected = _state;
    } else {
        _corrected = _state;
        //Integrate the displacement at corrected odometry state
        odometryInt(diff, _corrected);
    }
}
void OdometryModel::update(
    HumanoidFixedModel& model)
{
    update(model.get().getPose(), model.getSupportFoot());
}

const Eigen::VectorXd& OdometryModel::parameters() const
{
    return _odometryParameters;
}
Eigen::VectorXd& OdometryModel::parameters()
{
    return _odometryParameters;
}
        
const Eigen::Vector3d& OdometryModel::state() const
{
    return _corrected;
}

Eigen::Vector3d OdometryModel::odometryDiff(
    const Eigen::Vector3d& state1, 
    const Eigen::Vector3d& state2) const
{
    //Vector in world
    double vectX = state2.x() - state1.x();
    double vectY = state2.y() - state1.y();
    double angle = Leph::AngleDistance(state1.z(), state2.z()); 
    //Rotation to source frame
    double vectInSrcX = vectX*cos(-state1.z()) - vectY*sin(-state1.z());
    double vectInSrcY = vectX*sin(-state1.z()) + vectY*cos(-state1.z());

    return Eigen::Vector3d(vectInSrcX, vectInSrcY, angle);
}

void OdometryModel::odometryInt(
    const Eigen::Vector3d& diff,
    Eigen::Vector3d& state) const
{
    //Rotation to world frame
    double vectX = diff.x()*cos(state.z()) - diff.y()*sin(state.z());
    double vectY = diff.x()*sin(state.z()) + diff.y()*cos(state.z());
    //Integration
    state.x() += vectX;
    state.y() += vectY;
    state.z() += diff.z();
    //Shrink to -PI,PI
    if (state.z() > M_PI) state.z() -= 2.0*M_PI;
    if (state.z() < -M_PI) state.z() += 2.0*M_PI;
}

}

