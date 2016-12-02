#include <stdexcept>
#include <cmath>
#include "Model/JointModel.hpp"
#include "Utils/Angle.h"

namespace Leph {

JointModel::JointModel(
    JointModelType type, 
    const std::string& name) :
    _type(type),
    _name(name),
    _goalTime(0.0),
    _goalHistory(),
    //Backlash initial state
    _stateBacklashIsEnabled(true),
    _stateBacklashPosition(0.0),
    _stateBacklashVelocity(0.0),
    //Default parameters value
    //Friction velocity limit
    _paramFrictionVelLimit(0.2),
    //Joint internal gearbox inertia
    _paramInertiaIn(0.003),
    //Friction internal gearbox parameters
    _paramFrictionViscousIn(0.07),
    _paramFrictionBreakIn(0.08),
    _paramFrictionCoulombIn(0.05),
    //Joint external gearbox inertia
    _paramInertiaOut(0.001),
    //Friction external gearbox parameters
    _paramFrictionViscousOut(0.01),
    _paramFrictionBreakOut(0.05),
    _paramFrictionCoulombOut(0.02),
    //Electric motor voltage
    _paramElectricVoltage(12.0),
    //Electric motor ke
    _paramElectricKe(1.4),
    //Electric motor resistance
    _paramElectricResistance(4.07),
    //Motor proportional control
    _paramControlGainP(32.0),
    //Motor position discretization coef
    _paramControlDiscretization(2048.0),
    //Control lag in seconds
    _paramControlLag(0.030),
    //Backlash enable to disable position threshold
    _paramBacklashThresholdDeactivation(0.008),
    //Backlash disable to enable position threshold
    _paramBacklashThresholdActivation(0.006),
    //Backlash maximum angular distance
    _paramBacklashRangeMax(0.01)
{
    //Check joint type
    if (type != JointFree && type != JointActuated) {
        throw std::logic_error(
            "JointModel invalid joint type");
    }
}
        
JointModel::JointModelType JointModel::getType() const
{
    return _type;
}
        
const std::string& JointModel::getName() const
{
    return _name;
}

const Eigen::VectorXd JointModel::getParameters() const
{
    if (_type != JointActuated) {
        return Eigen::VectorXd();
    } 
    
    //Build parameters vector
    Eigen::VectorXd parameters(18);
    parameters(0) = _paramFrictionVelLimit;
    parameters(1) = _paramInertiaIn;
    parameters(2) = _paramFrictionViscousIn;
    parameters(3) = _paramFrictionBreakIn;
    parameters(4) = _paramFrictionCoulombIn;
    parameters(5) = _paramInertiaOut;
    parameters(6) = _paramFrictionViscousOut;
    parameters(7) = _paramFrictionBreakOut;
    parameters(8) = _paramFrictionCoulombOut;
    parameters(9) = _paramElectricKe;
    parameters(10) = _paramControlLag;
    parameters(11) = _paramBacklashThresholdDeactivation;
    parameters(12) = _paramBacklashThresholdActivation;
    parameters(13) = _paramBacklashRangeMax;
    parameters(14) = _paramElectricVoltage;
    parameters(15) = _paramElectricResistance;
    parameters(16) = _paramControlGainP;
    parameters(17) = _paramControlDiscretization;

    return parameters;
}
void JointModel::setParameters(const Eigen::VectorXd& parameters)
{
    if (_type != JointActuated) {
        return;
    }
    
    //Skrink to positive value
    Eigen::VectorXd tmpParams = parameters;
    for (size_t i=0;i<(size_t)tmpParams.size();i++) {
        if (tmpParams(i) < 0.0) {
            tmpParams(i) = 0.0;
        }
    }

    //Check size
    if (tmpParams.size() != 18) {
        throw std::logic_error(
            "JointModel invalid parameters size: "
            + std::to_string(tmpParams.size()));
    }

    //Assign parameters
    _paramFrictionVelLimit = tmpParams(0);
    _paramInertiaIn = tmpParams(1);
    _paramFrictionViscousIn = tmpParams(2);
    _paramFrictionBreakIn = tmpParams(3);
    _paramFrictionCoulombIn = tmpParams(4);
    _paramInertiaOut = tmpParams(5);
    _paramFrictionViscousOut = tmpParams(6);
    _paramFrictionBreakOut = tmpParams(7);
    _paramFrictionCoulombOut = tmpParams(8);
    _paramElectricKe = tmpParams(9);
    _paramControlLag = tmpParams(10);
    _paramBacklashThresholdDeactivation = tmpParams(11);
    _paramBacklashThresholdActivation = tmpParams(12);
    _paramBacklashRangeMax = tmpParams(13);
    _paramElectricVoltage = tmpParams(14);
    _paramElectricResistance = tmpParams(15);
    _paramControlGainP = tmpParams(16);
    _paramControlDiscretization = tmpParams(17);
}
        
double JointModel::getInertia() const
{
    if (_type != JointActuated) {
        return 0.0;
    }
    
    if (_stateBacklashIsEnabled) {
        return _paramInertiaOut;
    } else {
        return _paramInertiaIn + _paramInertiaOut;
    }
}

double JointModel::frictionTorque(double vel) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    if (_stateBacklashIsEnabled) {
        return computeFrictionTorque(
            vel, nullptr, false, true);
    } else {
        return computeFrictionTorque(
            vel, nullptr, true, true);
    }
}
        
double JointModel::controlTorque(double pos, double vel) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    if (_stateBacklashIsEnabled) {
        return 0.0;
    } else {
        return computeControlTorque(pos, vel);
    }
}

void JointModel::updateState(
    double dt, double goal, double pos, double vel)
{
    if (_type != JointActuated) {
        return;
    }

    //Append given goal
    _goalHistory.push({_goalTime, goal});
    //Update integrated time
    _goalTime += dt;

    //Pop history to get current goal lag
    while (
        _goalHistory.size() >= 2 &&
        _goalHistory.front().first < _goalTime - _paramControlLag
    ) {
        _goalHistory.pop();
    }

    //Compute backlash acceleration
    double backlashControlTorque = computeControlTorque(
        pos, vel);
    double backlashFrictionTorque = computeFrictionTorque(
        vel, &backlashControlTorque, true, false);
    double backlashAcc = 
        (backlashControlTorque + backlashFrictionTorque)/_paramInertiaIn;
    //Update backlash velocity and position
    double backlashNextVel = 
        _stateBacklashVelocity + dt*backlashAcc;
    _stateBacklashVelocity = 
        0.5*_stateBacklashVelocity + 0.5*backlashNextVel;
    _stateBacklashPosition = 
        _stateBacklashPosition + dt*_stateBacklashVelocity;
    //Compute bask
    //Update backlash state
    if (
        _stateBacklashIsEnabled && 
        fabs(AngleDistance(pos, _stateBacklashPosition)) > _paramBacklashThresholdDeactivation 
    ) {
        _stateBacklashIsEnabled = false;
    } else if (
        !_stateBacklashIsEnabled && 
        fabs(AngleDistance(pos, _stateBacklashPosition)) < _paramBacklashThresholdActivation
    ) {
        _stateBacklashIsEnabled = true;
    }
    //Bound backlash position
    if (AngleDistance(pos, _stateBacklashPosition) >= _paramBacklashRangeMax) {
        _stateBacklashPosition = pos + _paramBacklashRangeMax;
        _stateBacklashVelocity = 0.0;
    }
    if (AngleDistance(pos, _stateBacklashPosition) <= -_paramBacklashRangeMax) {
        _stateBacklashPosition = pos - _paramBacklashRangeMax;
        _stateBacklashVelocity = 0.0;
    }
    _stateBacklashPosition = AngleBound(_stateBacklashPosition);
}
        
double JointModel::getDelayedGoal() const
{
    if (_goalHistory.size() == 0) {
        return 0.0;
    } else {
        return _goalHistory.front().second;
    }
}
        
bool JointModel::getBacklashStateEnabled() const
{
    return _stateBacklashIsEnabled;
}
double JointModel::getBacklashStatePos() const
{
    return _stateBacklashPosition;
}
double JointModel::getBacklashStateVel() const
{
    return _stateBacklashVelocity;
}

void JointModel::boundState(double& pos, double& vel)
{
    //Check numerical instability
    if (fabs(pos) > 1e10 || fabs(vel) > 1e10) {
        throw std::runtime_error(
            "JointModel numerical instability:"
            + std::string(" name=") + _name
            + std::string(" pos=") + std::to_string(pos)
            + std::string(" vel=") + std::to_string(vel));
    }
    //Bound position angle inside [-PI:PI]
    if (_type == JointActuated) {
        pos = AngleBound(pos);
    }
}

double JointModel::ratioMaxControlTorque(
    double vel, double acc, double torque) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    //Compute current friction without backlash model
    double friction = computeFrictionTorque(
        vel, nullptr, true, true);
    if (fabs(vel) < 1e-6) {
        friction = 0.0;
    }

    //Compute the torque seens by the motor
    double motorTorque = 
        torque 
        + acc*(_paramInertiaIn + _paramInertiaOut) 
        - friction;
    if (fabs(motorTorque) < 1e-6) {
        return 0.0;
    }

    //Compute min or max control torque
    double boundMin = 0.0;
    double boundMax = 0.0;
    if (vel >= 0.0) {
        boundMax = 
            _paramElectricVoltage*_paramElectricKe/_paramElectricResistance 
            - vel/_paramElectricResistance;
        boundMin = 
            -_paramElectricVoltage*_paramElectricKe/_paramElectricResistance;
    } else {
        boundMax = 
            _paramElectricVoltage*_paramElectricKe/_paramElectricResistance;
        boundMin = 
            -_paramElectricVoltage*_paramElectricKe/_paramElectricResistance 
            - vel/_paramElectricResistance;
    }
    if (boundMax < 0.0) {
        boundMax = 0.0;
    }
    if (boundMin > 0.0) {
        boundMin = 0.0;
    }

    //Compute ratio
    if (motorTorque >= 0.0) {
        if (boundMax > 1e-6) {
            return motorTorque/boundMax;
        } else {
            return 10.0;
        }
    } else {
        if (boundMin < -1e-6) {
            return motorTorque/boundMin;
        } else {
            return 10.0;
        }
    }
}

double JointModel::computeFrictionTorque(
    double vel, double* torque, 
    bool isInFriction, bool isOutFriction) const
{
    //Apply internal and external friction model
    double usedFrictionVelLimit = _paramFrictionVelLimit;
    double usedFrictionViscous = 0.0;
    double usedFrictionBreak = 0.0;
    double usedFrictionCoulomb = 0.0;
    if (isInFriction) {
        usedFrictionViscous += _paramFrictionViscousIn;
        usedFrictionBreak += _paramFrictionBreakIn;
        usedFrictionCoulomb += _paramFrictionCoulombIn;
    }
    if (isOutFriction) {
        usedFrictionViscous += _paramFrictionViscousOut;
        usedFrictionBreak += _paramFrictionBreakOut;
        usedFrictionCoulomb += _paramFrictionCoulombOut;
    } 

    //Compute torque direction
    double sign;
    if (torque == nullptr || fabs(vel) > 1e-6) {
        sign = (vel >= 0.0 ? 1.0 : -1.0);
    } else if (torque != nullptr && fabs(*torque) > 1e-6) {
        sign = (*torque >= 0.0 ? 1.0 : -1.0);
    } else {
        //No information
        return 0.0;
    }

    //Compute friction
    double beta = exp(-fabs(vel/usedFrictionVelLimit));
    double forceViscous = -usedFrictionViscous*vel;
    double forceStatic1 = -sign*beta*usedFrictionBreak;
    double forceStatic2 = -sign*(1.0-beta)*usedFrictionCoulomb;

    return forceViscous + forceStatic1 + forceStatic2;
}
        
double JointModel::computeControlTorque(
    double pos, double vel) const
{
    //Apply position discretization
    double motorStepCoef = M_PI/_paramControlDiscretization;
    double discretizedPos = std::floor(pos/motorStepCoef)*motorStepCoef;

    //Retrieve delayed goal
    double delayedGoal = getDelayedGoal();

    //Compute current tension with
    //proportional controler
    double error = AngleDistance(discretizedPos, delayedGoal);
    double tension = _paramControlGainP*error
        *(4096.0/(2.0*M_PI))*(12.0/3000.0);

    //Bound to available voltage
    if (tension > _paramElectricVoltage) {
        tension = _paramElectricVoltage;
    } else if (tension < -_paramElectricVoltage) {
        tension = -_paramElectricVoltage;
    }

    //Compute applied electrical torque
    double torque = 
        tension*_paramElectricKe/_paramElectricResistance 
        - vel/_paramElectricResistance;
    
    return torque;
}
        
}

