#include <stdexcept>
#include "Model/JointModel.hpp"

namespace Leph {

JointModel::JointModel(
    JointModelType type, 
    const std::string& name) :
    _type(type),
    _name(name),
    _parameters(),
    _state()
{
    //Initialize model parameters
    if (type == JointFree) {
        //No friction, no control
        _parameters = Eigen::VectorXd();
        _state = Eigen::VectorXd();
    } else if (type == JointActuated) {
        //Friction and control
        _parameters = Eigen::VectorXd(8);
        _state = Eigen::VectorXd();
        //Friction velocity limit
        _parameters(0) = 0.2;
        //Friction viscous
        _parameters(1) = 0.2;
        //Friction static Coulomb
        _parameters(2) = 1.0;
        //Friction static breakaway
        _parameters(3) = 1.0;
        //Control proportional gain
        _parameters(4) = 80.0;
        //Control max torque at zero velocity
        _parameters(5) = 40.0;
        //Control max velocity at zero torque
        _parameters(6) = 50.0;
        //Control lag in seconds
        _parameters(7) = 0.00;
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
    double pos, double vel)
{
    if (_type == JointActuated) {
        double coefVelLimit = _parameters(0);
        double coefViscous = _parameters(1);
        double coefStatic = _parameters(2);
        double coefBreak = _parameters(3);
        if (fabs(vel) < 0.001) {
            return 0.0;
        }
        double sign = (vel >= 0.0 ? 1.0 : -1.0);
        double beta = exp(-fabs(vel/coefVelLimit));
        return -coefViscous*vel 
            - sign*(beta*coefBreak + (1.0-beta)*coefStatic);
    } else {
        return 0.0;
    }
}
        
double JointModel::controlTorque(
    double dt, double goal, 
    double pos, double vel)
{
    if (_type != JointActuated) {
        return 0.0;
    }

    //Retrieve parameters
    double gainP = _parameters(4);
    double maxTorque = _parameters(5);
    double maxVel = _parameters(6);
    double lag = _parameters(7);

    //Increment current time
    _timeCounter += dt;

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

    //Time sanity check
    if (_history.size() > 0 && _history.back().first >= _timeCounter) {
        throw std::logic_error(
            "JointModel history timestamp error");
    }
    //Append current target goal to history
    _history.push_back({_timeCounter, goal});
    //Used lagged time
    double pastTime = _timeCounter - lag;
    double delayedGoal;
    //Compute delayed goal
    if (_history.size() == 1 || _history.front().first >= pastTime) {
        delayedGoal = _history.front().second;
    } else {
        //Pop history to meet lagged time
        while (_history.size() >= 2 && _history[1].first < pastTime) {
            _history.pop_front();
        }
        //Linear goal interpolation
        double tsLow = _history[0].first;
        double valLow = _history[0].second;
        double tsUp = _history[1].first;
        double valUp = _history[1].second;
        delayedGoal = 
            (tsUp-pastTime)/(tsUp-tsLow)*valLow 
            + (pastTime-tsLow)/(tsUp-tsLow)*valUp;
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
        
void JointModel::boundState(double& pos, double& vel)
{
    //Check numerical instability
    if (fabs(pos) > 1e10 || fabs(vel) > 1e10) {
        throw std::runtime_error(
            "JointModel numerical instability");
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
        
const Eigen::VectorXd& JointModel::getState() const
{
    return _state;
}

}

