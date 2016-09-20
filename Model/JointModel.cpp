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
    _states(),
    _isInitialized(false)
{
    //Initialize model parameters
    if (type == JointFree) {
        //No friction, no control
        _parameters = Eigen::VectorXd();
        _states = Eigen::VectorXd();
    } else if (type == JointActuated) {
        //Friction and control
        _parameters = Eigen::VectorXd(9);
        _states = Eigen::VectorXd(1);
        //Friction velocity limit
        _parameters(0) = 0.1;
        //Friction viscous
        _parameters(1) = 0.05;
        //Friction static Coulomb
        _parameters(2) = 0.06;
        //Friction static breakaway
        _parameters(3) = 0.08;
        //Control proportional gain
        _parameters(4) = 60.0;
        //Control max torque at zero velocity
        _parameters(5) = 40.0;
        //Control max velocity at zero torque
        _parameters(6) = 50.0;
        //Control lag in seconds
        _parameters(7) = 0.0;
        //Control delayed goal mult
        _parameters(8) = 1.0;
        //Current delayed goal
        _states(0) = 0.0;
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

const Eigen::VectorXd& JointModel::getStates() const
{
    return _states;
}
void JointModel::setStates(const Eigen::VectorXd& states)
{
    if (states.size() != _states.size()) {
        throw std::logic_error(
            "JointModel invalid states size: "
            + std::to_string(states.size()));
    }
    _states = states;
}
        
double JointModel::frictionTorque(double pos, double vel) const
{
    (void)pos;
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double coefVelLimit = _parameters(0);
    double coefViscous = _parameters(1);
    double coefStatic = _parameters(2);
    double coefBreak = _parameters(3);

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
    double gainP = _parameters(4);
    double maxTorque = _parameters(5);
    double maxVel = _parameters(6);

    //Retrieve discounted goal
    double delayedGoal = _states(0);

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
    if (!_isInitialized) {
        _isInitialized = true;
        _states(0) = pos;
        _states(1) = pos;
    }


    //Retrive parameter
    double lag = _parameters(7);
    double mult = _parameters(8);
    
    //Update delayed target goal
    if (fabs(lag) > 1e-6) {
        _states(0) = _states(0) + mult*dt*(goal-_states(0))/lag;
    } else {
        _states(0) = goal;
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

