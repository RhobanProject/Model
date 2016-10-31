#include <stdexcept>
#include <cmath>
#include "Model/JointModel.hpp"

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
        _parameters = Eigen::VectorXd(11);
        //Joint internal inertia
        _parameters(0) = 0.00353;
        //Friction velocity limit
        _parameters(1) = 0.195;
        //Friction viscous
        _parameters(2) = 0.5;
        //Friction static Coulomb
        _parameters(3) = 0.2;
        //Friction static breakaway
        _parameters(4) = 0.3;
        //Control proportional gain
        _parameters(5) = 45.0;
        //Control max torque at zero velocity
        _parameters(6) = 6.0;
        //Control max velocity at zero torque
        _parameters(7) = 10.0;
        //Control lag in seconds
        _parameters(8) = 0.030;
        //Backlask angular range
        _parameters(9) = 0.01;
        //Backlash friction coef
        _parameters(10) = 0.02;
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
    double backlashRange = _parameters(9);
    double backlashCoef = _parameters(10);

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

    //Retrieve discounted goal
    double delayedGoal = getDelayedGoal();

    //Compute min and max torque range
    double rangeMax = controlMaxTorque(vel);
    double rangeMin = controlMinTorque(vel);
    //Assert check
    if (rangeMax < rangeMin || 
        rangeMax < 0.0 || 
        rangeMin > 0.0
    ) {
        throw std::logic_error(
            "JointModel control torque range invalid");
    }

    //Compute current control torque
    double control = gainP*(delayedGoal - pos);

    //Bound generated torque inside range
    if (control > rangeMax) {
        control = rangeMax;
    }
    if (control < rangeMin) {
        control = rangeMin;
    }

    return control;
}

double JointModel::controlMaxTorque(double vel) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double maxTorque = _parameters(6);
    double maxVel = _parameters(7);
    
    //Compute maximum bound
    if (vel >= 0.0) {
        return std::max(0.0, 
            -vel*maxTorque/maxVel + maxTorque);
    } else {
        return maxTorque;
    }
}
double JointModel::controlMinTorque(double vel) const
{
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double maxTorque = _parameters(6);
    double maxVel = _parameters(7);
    
    //Compute minimum bound
    if (vel >= 0.0) {
        return -maxTorque;
    } else {
        return -std::max(0.0, 
            vel*maxTorque/maxVel + maxTorque);
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
    double delay = _parameters(8);
    while (
        _goalHistory.size() >= 2 &&
        _goalHistory.front().first < _goalTime - delay
    ) {
        _goalHistory.pop();
    }

    //Backlash state update
    double backlashRange = _parameters(9);
    _backlashPosition += dt*vel;
    if (_backlashPosition > backlashRange) {
        _backlashPosition = backlashRange + 1e-10;
    }
    if (_backlashPosition < -backlashRange) {
        _backlashPosition = -backlashRange - 1e-10;
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

    //Retrieve internal inertia parameter
    double inertia = _parameters(0);

    //Compute current friction with backlash model
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
    double bound;
    if (motorTorque >= 0.0) {
        bound = controlMaxTorque(vel);
    } else {
        bound = controlMinTorque(vel);
    }
    
    //Compute and return the ratio
    if (fabs(bound) < 1e-6) {
        return 10.0;
    } else {
        return motorTorque/bound;
    }
}
        
}

