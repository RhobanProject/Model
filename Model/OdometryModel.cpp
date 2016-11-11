#include <stdexcept>
#include <Utils/Angle.h>
#include "Model/OdometryModel.hpp"

namespace Leph {
        
OdometryModel::OdometryModel(OdometryModelType type) :
    _type(type),
    _isInitialized(false),
    _odometryParameters(),
    _odometryLowerBounds(),
    _odometryUpperBounds(),
    _support(),
    _last(),
    _state(),
    _corrected()
{
    double identityLowerBound = 0.5;
    double identityUpperBound = 2.0;
    double offsetStepBound = 0.02;
    double offsetLateraBound = 0.02;
    double offsetTurnBound = 0.05;
    double coefStepLateralBound = 10.0;
    double coefStepTurnBound = 10.0;
    double coefLateralStepBound = 10.0;
    double coefLateralTurnBound = 10.0;
    double coefTurnStepBound = 10.0;
    double coefTurnLateralBound = 10.0;
    if (_type == CorrectionIdentity) {
        //Dummy correction
        _odometryParameters = Eigen::VectorXd::Zero(0);
        _odometryLowerBounds = Eigen::VectorXd::Zero(0);
        _odometryUpperBounds = Eigen::VectorXd::Zero(0);
    } else if (_type == CorrectionScalarX) {
        _odometryParameters = Eigen::VectorXd::Zero(1);
        _odometryLowerBounds = Eigen::VectorXd::Zero(1);
        _odometryUpperBounds = Eigen::VectorXd::Zero(1);
        _odometryParameters <<
            1.0; //dX = dx
        _odometryLowerBounds <<
            identityLowerBound; //dX = dx
        _odometryUpperBounds <<
            identityUpperBound; //dX = dx
    } else if (_type == CorrectionScalarXY) {
        _odometryParameters = Eigen::VectorXd::Zero(1);
        _odometryLowerBounds = Eigen::VectorXd::Zero(1);
        _odometryUpperBounds = Eigen::VectorXd::Zero(1);
        _odometryParameters <<
            1.0; //dX = dx, dY = dy
        _odometryLowerBounds <<
            identityLowerBound; //dX = dx, dY = dy
        _odometryUpperBounds <<
            identityUpperBound; //dX = dx, dY = dy
    } else if (_type == CorrectionScalarXYZ) {
        _odometryParameters = Eigen::VectorXd::Zero(1);
        _odometryLowerBounds = Eigen::VectorXd::Zero(1);
        _odometryUpperBounds = Eigen::VectorXd::Zero(1);
        _odometryParameters <<
            1.0; //dX = dx, dY = dy, dA = da
        _odometryLowerBounds <<
            identityLowerBound; //dX = dx, dY = dy, dA = da
        _odometryUpperBounds <<
            identityUpperBound; //dX = dx, dY = dy, dA = da
    } else if (_type == CorrectionProportionalXY) {
        _odometryParameters = Eigen::VectorXd::Zero(2);
        _odometryLowerBounds = Eigen::VectorXd::Zero(2);
        _odometryUpperBounds = Eigen::VectorXd::Zero(2);
        _odometryParameters <<
            1.0, //dX = dx
            1.0; //dY = dy
        _odometryLowerBounds <<
            identityLowerBound, //dX = dx
            identityLowerBound; //dY = dy
        _odometryUpperBounds <<
            identityUpperBound, //dX = dx
            identityUpperBound; //dY = dy
    } else if (_type == CorrectionProportionalXYZ) {
        _odometryParameters = Eigen::VectorXd::Zero(3);
        _odometryLowerBounds = Eigen::VectorXd::Zero(3);
        _odometryUpperBounds = Eigen::VectorXd::Zero(3);
        _odometryParameters <<
            1.0, //dX = dx
            1.0, //dY = dy
            1.0; //dA = da
        _odometryLowerBounds <<
            identityLowerBound, //dX = dx
            identityLowerBound, //dY = dy
            identityLowerBound; //dA = da
        _odometryUpperBounds <<
            identityUpperBound, //dX = dx
            identityUpperBound, //dY = dy
            identityUpperBound; //dA = da
    } else if (_type == CorrectionLinearSimpleXY) {
        _odometryParameters = Eigen::VectorXd::Zero(4);
        _odometryLowerBounds = Eigen::VectorXd::Zero(4);
        _odometryUpperBounds = Eigen::VectorXd::Zero(4);
        _odometryParameters <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dY = offset
            1.0; //dY = dy
        _odometryLowerBounds <<
            -offsetStepBound, //dX = offset
            identityLowerBound,  //dX = dx
            -offsetLateraBound, //dY = offset
            identityLowerBound;  //dY = dy
        _odometryUpperBounds <<
            offsetStepBound, //dX = offset
            identityUpperBound, //dX = dx
            offsetLateraBound, //dY = offset
            identityUpperBound; //dY = dy
    } else if (_type == CorrectionLinearSimpleXYZ) {
        _odometryParameters = Eigen::VectorXd::Zero(6);
        _odometryLowerBounds = Eigen::VectorXd::Zero(6);
        _odometryUpperBounds = Eigen::VectorXd::Zero(6);
        _odometryParameters <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dY = offset
            1.0, //dY = dy
            0.0, //dA = offset
            1.0; //dA = da
        _odometryLowerBounds <<
            -offsetStepBound, //dX = offset
            identityLowerBound,  //dX = dx
            -offsetLateraBound, //dY = offset
            identityLowerBound,  //dY = dy
            -offsetTurnBound, //dA = offset
            identityLowerBound;  //dA = da
        _odometryUpperBounds <<
            offsetStepBound, //dX = offset
            identityUpperBound, //dX = dx
            offsetLateraBound, //dY = offset
            identityUpperBound, //dY = dy
            offsetTurnBound, //dA = offset
            identityUpperBound; //dA = da
    } else if (_type == CorrectionLinearFullXY) {
        _odometryParameters = Eigen::VectorXd::Zero(8);
        _odometryLowerBounds = Eigen::VectorXd::Zero(8);
        _odometryUpperBounds = Eigen::VectorXd::Zero(8);
        _odometryParameters <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dX = dy
            0.0, //dX = da
            0.0, //dY = offset
            0.0, //dY = dx
            1.0, //dY = dy
            0.0; //dY = da
        _odometryLowerBounds <<
            -offsetStepBound, //dX = offset
            identityLowerBound,   //dX = dx
            -coefStepLateralBound,  //dX = dy
            -coefStepTurnBound,  //dX = da
            -offsetLateraBound, //dY = offset
            -coefLateralStepBound,  //dY = dx
            identityLowerBound,   //dY = dy
            -coefLateralTurnBound;  //dY = da
        _odometryUpperBounds <<
            offsetStepBound, //dX = offset
            identityUpperBound,  //dX = dx
            coefStepLateralBound,  //dX = dy
            coefStepTurnBound,  //dX = da
            offsetLateraBound, //dY = offset
            coefLateralStepBound,  //dY = dx
            identityUpperBound,  //dY = dy
            coefLateralTurnBound;  //dY = da
    } else if (_type == CorrectionLinearFullXYZ) {
        _odometryParameters = Eigen::VectorXd::Zero(12);
        _odometryLowerBounds = Eigen::VectorXd::Zero(12);
        _odometryUpperBounds = Eigen::VectorXd::Zero(12);
        _odometryParameters <<
            0.0, //dX = offset
            1.0, //dX = dx
            0.0, //dX = dy
            0.0, //dX = da
            0.0, //dY = offset
            0.0, //dY = dx
            1.0, //dY = dy
            0.0, //dY = da
            0.0, //dA = offset
            0.0, //dA = dx
            0.0, //dA = dy
            1.0; //dA = da
        _odometryLowerBounds <<
            -offsetStepBound, //dX = offset
            identityLowerBound,   //dX = dx
            -coefStepLateralBound,  //dX = dy
            -coefStepTurnBound,  //dX = da
            -offsetLateraBound, //dY = offset
            -coefLateralStepBound,  //dY = dx
            identityLowerBound,   //dY = dy
            -coefLateralTurnBound,  //dY = da
            -offsetTurnBound, //dA = offset
            -coefTurnStepBound,  //dA = dx
            -coefTurnLateralBound,   //dA = dy
            identityLowerBound;  //dA = da
        _odometryUpperBounds <<
            offsetStepBound, //dX = offset
            identityUpperBound,  //dX = dx
            coefStepLateralBound,  //dX = dy
            coefStepTurnBound,  //dX = da
            offsetLateraBound, //dY = offset
            coefLateralStepBound,  //dY = dx
            identityUpperBound,  //dY = dy
            coefLateralTurnBound,  //dY = da
            offsetTurnBound, //dA = offset
            coefTurnStepBound,  //dA = dx
            coefTurnLateralBound,  //dA = dy
            identityUpperBound;  //dA = da
    } else {
        throw std::logic_error(
            "OdometryModel invalid type");
    }
    //Ask reset
    reset();
}
        
OdometryModel::OdometryModelType OdometryModel::getType() const
{
    return _type;
}
        
void OdometryModel::reset()
{
    _isInitialized = false;
    _state = Eigen::Vector3d(0.0, 0.0, 0.0);
    _corrected = Eigen::Vector3d(0.0, 0.0, 0.0);
}
void OdometryModel::reset(const Eigen::Vector3d& pose)
{
    _isInitialized = false;
    _state = pose;
    _corrected = pose;
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
    diff = correctiveModel(diff);

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
        
void OdometryModel::updateFullStep(
    const Eigen::Vector3d& deltaPose)
{
    if (!_isInitialized) {
        _support = Leph::HumanoidFixedModel::LeftSupportFoot;
        _last = _state;
        _isInitialized = true;
    }

    //Apply displacement correction
    Eigen::Vector3d diff = correctiveModel(deltaPose);
    
    //Integrate current state with given full 
    //step relative displacement
    _last = _state;
    odometryInt(diff, _state);
    _corrected = _state;
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
        
Eigen::Vector3d OdometryModel::correctiveModel(
    const Eigen::Vector3d& diff) const
{
    double diffX = diff.x();
    double diffY = diff.y();
    double diffZ = diff.z();
    if (_type == CorrectionIdentity) {
        diffX = diff.x();
        diffY = diff.y();
        diffZ = diff.z();
    } else if (_type == CorrectionScalarX) {
        diffX = 
            _odometryParameters(0)*diff.x();
    } else if (_type == CorrectionScalarXY) {
        diffX = 
            _odometryParameters(0)*diff.x();
        diffY = 
            _odometryParameters(0)*diff.y();
    } else if (_type == CorrectionScalarXYZ) {
        diffX = 
            _odometryParameters(0)*diff.x();
        diffY = 
            _odometryParameters(0)*diff.y();
        diffZ = 
            _odometryParameters(0)*diff.z();
    } else if (_type == CorrectionProportionalXY) {
        diffX = 
            _odometryParameters(0)*diff.x();
        diffY = 
            _odometryParameters(1)*diff.y();
    } else if (_type == CorrectionProportionalXYZ) {
        diffX = 
            _odometryParameters(0)*diff.x();
        diffY = 
            _odometryParameters(1)*diff.y();
        diffZ = 
            _odometryParameters(2)*diff.z();
    } else if (_type == CorrectionLinearSimpleXY) {
        diffX = 
            _odometryParameters(0) +
            _odometryParameters(1)*diff.x();
        diffY = 
            _odometryParameters(2) +
            _odometryParameters(3)*diff.y();
    } else if (_type == CorrectionLinearSimpleXYZ) {
        diffX = 
            _odometryParameters(0) +
            _odometryParameters(1)*diff.x();
        diffY = 
            _odometryParameters(2) +
            _odometryParameters(3)*diff.y();
        diffZ = 
            _odometryParameters(4) +
            _odometryParameters(5)*diff.z();
    } else if (_type == CorrectionLinearFullXY) {
        diffX = 
            _odometryParameters(0) +
            _odometryParameters(1)*diff.x() +
            _odometryParameters(2)*diff.y() +
            _odometryParameters(3)*diff.z();
        diffY = 
            _odometryParameters(4) +
            _odometryParameters(5)*diff.x() + 
            _odometryParameters(6)*diff.y() +
            _odometryParameters(7)*diff.z();
    } else if (_type == CorrectionLinearFullXYZ) {
        diffX = 
            _odometryParameters(0) +
            _odometryParameters(1)*diff.x() +
            _odometryParameters(2)*diff.y() +
            _odometryParameters(3)*diff.z();
        diffY = 
            _odometryParameters(4) +
            _odometryParameters(5)*diff.x() +
            _odometryParameters(6)*diff.y() +
            _odometryParameters(7)*diff.z();
        diffZ = 
            _odometryParameters(8) +
            _odometryParameters(9)*diff.x() +
            _odometryParameters(10)*diff.y() +
            _odometryParameters(11)*diff.z();
    } else {
        throw std::logic_error(
            "OdometryModel invalid type");
    }

    return Eigen::Vector3d(diffX, diffY, diffZ);
}

const Eigen::VectorXd& OdometryModel::parameterLowerBounds() const
{
    return _odometryLowerBounds;
}
const Eigen::VectorXd& OdometryModel::parameterUpperBounds() const
{
    return _odometryUpperBounds;
}

}

