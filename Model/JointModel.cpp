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
    _isInitialized(false),
    //Backlash initial state
    _stateBacklashIsEnabled(true),
    _stateBacklashPosition(0.0),
    _stateBacklashVelocity(0.0),
    //Firmware coeficients
    _coefAnglePosToTension((4096.0/(2.0*M_PI))*(12.0/3000.0)),
    //Default parameters value
    //Friction velocity limit
    _paramFrictionVelLimit(0.2),
    //Joint internal gearbox inertia
    _paramInertiaIn(0.003),
    //Friction internal gearbox parameters
    //(BreakIn is a positive offset above CoulombIn)
    _paramFrictionViscousIn(0.07),
    _paramFrictionBreakIn(0.03),
    _paramFrictionCoulombIn(0.05),
    //Joint external gearbox inertia
    _paramInertiaOut(0.001),
    //Friction external gearbox parameters
    //(BreakOut is a positive offset above CoulombOut)
    _paramFrictionViscousOut(0.01),
    _paramFrictionBreakOut(0.03),
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
    //(Positive offset above activation)
    _paramBacklashThresholdDeactivation(0.002),
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

    //Temporary identified parameters
    _paramFrictionVelLimit = 0.2914428092;
    _paramFrictionViscousOut = 0.003099660986;
    _paramFrictionBreakOut = 0.04943333614;
    _paramFrictionCoulombOut = 0.02171844766;
    _paramInertiaOut = 0.0007936561532;
    _paramControlLag = 0.03300551425;
    _paramFrictionViscousIn = 0.1242307908;
    _paramFrictionBreakIn = 0.10865058966;
    _paramFrictionCoulombIn = 0.07522603114;
    _paramInertiaIn = 0.005212544434;
    _paramBacklashRangeMax = 0.006733195882;
    _paramBacklashThresholdDeactivation = 0.002;
    _paramBacklashThresholdActivation = 0.004;
    _paramElectricKe = 1.951435691;
    _paramElectricVoltage = 12.65527041;
    _paramElectricResistance = 3.098613714;
    _paramControlGainP = 39.32076467;
    _paramControlDiscretization = 2048.0;
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
    parameters(1) = _paramFrictionViscousOut;
    parameters(2) = _paramFrictionBreakOut;
    parameters(3) = _paramFrictionCoulombOut;
    parameters(4) = _paramInertiaOut;
    parameters(5) = _paramControlLag;
    parameters(6) = _paramFrictionViscousIn;
    parameters(7) = _paramFrictionBreakIn;
    parameters(8) = _paramFrictionCoulombIn;
    parameters(9) = _paramInertiaIn;
    parameters(10) = _paramBacklashRangeMax;
    parameters(11) = _paramBacklashThresholdDeactivation;
    parameters(12) = _paramBacklashThresholdActivation;
    parameters(13) = _paramElectricKe;
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
    _paramFrictionViscousOut = tmpParams(1);
    _paramFrictionBreakOut = tmpParams(2);
    _paramFrictionCoulombOut = tmpParams(3);
    _paramInertiaOut = tmpParams(4);
    _paramControlLag = tmpParams(5);
    _paramFrictionViscousIn = tmpParams(6);
    _paramFrictionBreakIn = tmpParams(7);
    _paramFrictionCoulombIn = tmpParams(8);
    _paramInertiaIn = tmpParams(9);
    _paramBacklashRangeMax = tmpParams(10);
    _paramBacklashThresholdDeactivation = tmpParams(11);
    _paramBacklashThresholdActivation = tmpParams(12);
    _paramElectricKe = tmpParams(13);
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
        
double JointModel::getMaxVoltage() const
{
    return _paramElectricVoltage;
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

    //Hidden state initialization
    if (!_isInitialized) {
        _stateBacklashIsEnabled = true;
        _stateBacklashPosition = pos;
        _stateBacklashVelocity = vel;
        _isInitialized = true;
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
    double relativePos = fabs(AngleDistance(pos, _stateBacklashPosition));
    //Used state transition thresholds
    double usedThresholdDeactivation = 
        _paramBacklashThresholdDeactivation 
        + _paramBacklashThresholdActivation;
    double usedThresholdActivation = _paramBacklashThresholdActivation;
    if (
        _stateBacklashIsEnabled && 
        relativePos > usedThresholdDeactivation
    ) {
        _stateBacklashIsEnabled = false;
    } else if (
        !_stateBacklashIsEnabled && 
        relativePos < usedThresholdActivation
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

double JointModel::computeElectricTension(
    double velGoal,
    double accGoal,
    double torqueGoal) const
{
    //Compute friction torque 
    //(backlask is not considered)
    double frictionTorque = computeFrictionTorque(
        velGoal, &torqueGoal, true, true);

    //Compute torque from gears inertia
    //(backlask is not considered)
    double inertiaTorque = accGoal*(_paramInertiaIn + _paramInertiaOut);

    //Compute internal torque seen by the motor
    //to produce expected motion
    double torqueInternalGoal = 
        torqueGoal - frictionTorque + inertiaTorque;

    //Compute expected electric tension to produce
    //needed torque at motor output
    double tensionGoal = 
        torqueInternalGoal*_paramElectricResistance/_paramElectricKe
        + velGoal*_paramElectricResistance;

    return tensionGoal;
}
        
double JointModel::computeFeedForward(
    double velGoal,
    double accGoal,
    double torqueGoal) const
{
    //Compute expected motor tension
    double tensionGoal = computeElectricTension(
        velGoal, accGoal, torqueGoal);

    //Bound tension to controller capability
    if (tensionGoal > _paramElectricVoltage) {
        tensionGoal = _paramElectricVoltage;
    }
    if (tensionGoal < -_paramElectricVoltage) {
        tensionGoal = -_paramElectricVoltage;
    }

    //Compute angular offset from tension
    double angularOffset = 
        tensionGoal/(_paramControlGainP*_coefAnglePosToTension);

    return angularOffset;
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
        usedFrictionBreak += 
            _paramFrictionBreakIn + _paramFrictionCoulombIn;
        usedFrictionCoulomb += _paramFrictionCoulombIn;
    }
    if (isOutFriction) {
        usedFrictionViscous += _paramFrictionViscousOut;
        usedFrictionBreak += 
            _paramFrictionBreakOut + _paramFrictionCoulombOut;
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
    double tension = _paramControlGainP*error*_coefAnglePosToTension;

    //Bound to available voltage
    if (tension > _paramElectricVoltage) {
        tension = _paramElectricVoltage;
    } else if (tension < -_paramElectricVoltage) {
        tension = -_paramElectricVoltage;
    }

    //Compute applied electrical torque
    double torque = 
        tension*_paramElectricKe/_paramElectricResistance 
        - vel*pow(_paramElectricKe, 2)/_paramElectricResistance;
    
    return torque;
}
        
}

