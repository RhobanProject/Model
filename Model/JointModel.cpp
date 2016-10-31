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
    _parameters(),
    _goalTime(0.0),
    _goalHistory(),
    _backlashPosition(0.0)
{
    //Initialize model parameters
    if (type == JointFree) {
        //No friction, no control
        _parameters = Eigen::VectorXd();
    } else if (type == JointActuated) {
        //Friction and control
        _parameters = Eigen::VectorXd(12);
        //Joint internal inertia
        _parameters(0) = 0.00353;
        //Friction velocity limit
        _parameters(1) = 0.195;
        //Friction viscous
        _parameters(2) = 0.0811;
        //Friction static Coulomb
        _parameters(3) = 0.0673;
        //Friction static breakaway
        _parameters(4) = 0.1212;
        //Control proportional gain
        _parameters(5) = 32.0;
        //Electric motor voltage
        _parameters(6) = 12.0;
        //Electric motor ke
        _parameters(7) = 1.399;
        //Electric motor resistance
        _parameters(8) = 4.07;
        //Control lag in seconds
        _parameters(9) = 0.030;
        //Backlask angular range
        _parameters(10) = 0.005;
        //Backlash friction coef
        _parameters(11) = 0.2;
    } else {
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

const Eigen::VectorXd& JointModel::getParameters() const
{
    return _parameters;
}
void JointModel::setParameters(const Eigen::VectorXd& params)
{
    if (params.size() != _parameters.size()) {
        throw std::logic_error(
            "JointModel invalid parameters size: "
            + std::to_string(params.size()));
    }
    _parameters = params;

    //Skrink to positive value
    for (size_t i=0;i<(size_t)_parameters.size();i++) {
        if (_parameters(i) < 0.0) {
            _parameters(i) = 0.0;
        }
    }
}
        
double JointModel::getInertia() const
{
    if (_type != JointActuated) {
        return 0.0;
    } else {
        return _parameters(0);
    }
}

double JointModel::frictionTorque(
    double vel, bool isBacklash) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double coefVelLimit = _parameters(1);
    double coefViscous = _parameters(2);
    double coefStatic = _parameters(3);
    double coefBreak = _parameters(4);
    double backlashRange = _parameters(10);
    double backlashCoef = _parameters(11);

    //Apply backlash friction reduction
    if (
        isBacklash && 
        fabs(_backlashPosition) < backlashRange
    ) {
        coefViscous *= backlashCoef;
        coefStatic *= backlashCoef;
        coefBreak *= backlashCoef;
    }

    //Compute friction
    double sign = (vel >= 0.0 ? 1.0 : -1.0);
    double beta = exp(-fabs(vel/coefVelLimit));
    double forceViscous = -coefViscous*vel;
    double forceStatic1 = -sign*(beta*coefBreak);
    double forceStatic2 = -sign*(1.0-beta)*coefStatic;

    return forceViscous + forceStatic1 + forceStatic2;
}
        
double JointModel::controlTorque(double pos, double vel) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double gainP = _parameters(5);
    double voltageMax = _parameters(6);
    double k_e = _parameters(7);
    double resistance = _parameters(8);

    //Retrieve discounted goal
    double delayedGoal = getDelayedGoal();

    //Compute current tension with
    //proportional controler
    double error = AngleDistance(pos, delayedGoal);
    double tension = gainP*error
        *(4096.0/(2.0*M_PI))*(12.0/3000.0);
    //Bound to available voltage
    if (tension > voltageMax) {
        tension = voltageMax;
    } else if (tension < -voltageMax) {
        tension = -voltageMax;
    }

    //Compute applied electrical torque
    double torque = tension*k_e/resistance - vel/resistance;
    return torque;
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
    double delay = _parameters(9);
    while (
        _goalHistory.size() >= 2 &&
        _goalHistory.front().first < _goalTime - delay
    ) {
        _goalHistory.pop();
    }

    //Backlash state update
    double backlashRange = _parameters(10);
    _backlashPosition += dt*vel;
    if (_backlashPosition >= backlashRange) {
        _backlashPosition = backlashRange + 1e-9;
    }
    if (_backlashPosition <= -backlashRange) {
        _backlashPosition = -backlashRange - 1e-9;
    }
}
        
double JointModel::getDelayedGoal() const
{
    if (_goalHistory.size() == 0) {
        return 0.0;
    } else {
        return _goalHistory.front().second;
    }
}
        
double JointModel::getBacklashState() const
{
    return _backlashPosition;
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
        while (pos > M_PI) {
            pos -= 2.0*M_PI;
        }
        while (pos < -M_PI) {
            pos += 2.0*M_PI;
        }
    }
}

double JointModel::ratioMaxControlTorque(
    double vel, double acc, double torque) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double inertia = _parameters(0);
    double voltageMax = _parameters(6);
    double k_e = _parameters(7);
    double resistance = _parameters(8);

    //Compute current friction without backlash model
    double friction = frictionTorque(vel, false);
    if (fabs(vel) < 1e-6) {
        friction = 0.0;
    }

    //Compute the torque seens by the motor
    double motorTorque = torque + acc*inertia - friction;
    if (fabs(motorTorque) < 1e-6) {
        return 0.0;
    }

    //Compute min or max control torque
    double boundMin = 0.0;
    double boundMax = 0.0;
    if (vel >= 0.0) {
        boundMax = voltageMax*k_e/resistance - vel/resistance;
        boundMin = -voltageMax*k_e/resistance;
    } else {
        boundMax = voltageMax*k_e/resistance;
        boundMin = -voltageMax*k_e/resistance - vel/resistance;
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
        
}

