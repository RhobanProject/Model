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
    _goalHistory()
{
    //Initialize model parameters
    if (type == JointFree) {
        //No friction, no control
        _parameters = Eigen::VectorXd();
    } else if (type == JointActuated) {
        //Friction and control
        _parameters = Eigen::VectorXd(9);
        //Joint internal inertia
        _parameters(0) = 0.01;
        //Friction velocity limit
        _parameters(1) = 0.1;
        //Friction viscous
        _parameters(2) = 0.05;
        //Friction static Coulomb
        _parameters(3) = 0.06;
        //Friction static breakaway
        _parameters(4) = 0.08;
        //Control proportional gain
        _parameters(5) = 60.0;
        //Control max torque at zero velocity
        _parameters(6) = 40.0;
        //Control max velocity at zero torque
        _parameters(7) = 50.0;
        //Control lag in seconds
        _parameters(8) = 0.01;
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

double JointModel::frictionTorque(double pos, double vel) const
{
    (void)pos;
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double coefVelLimit = _parameters(1);
    double coefViscous = _parameters(2);
    double coefStatic = _parameters(3);
    double coefBreak = _parameters(4);

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
    double maxTorque = _parameters(6);
    double maxVel = _parameters(7);

    //Retrieve discounted goal
    double delayedGoal = getDelayedGoal();

    //Compute min and max torque range
    double rangeMax;
    double rangeMin;
    if (vel >= 0.0) {
        rangeMax = std::max(0.0, 
            -vel*maxTorque/maxVel + maxTorque);
        rangeMin = -maxTorque;
    } else {
        rangeMax = maxTorque;
        rangeMin = -std::max(0.0, 
            vel*maxTorque/maxVel + maxTorque);
    }
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
}
        
double JointModel::getDelayedGoal() const
{
    if (_goalHistory.size() == 0) {
        return 0.0;
    } else {
        return _goalHistory.front().second;
    }
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
        
}

