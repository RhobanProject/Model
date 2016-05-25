#include "Model/MotorModel.hpp"

namespace Leph {
        
double MotorModel::maxVoltage()
{
    return _uMax;
}
        
double MotorModel::maxTorque()
{
    return _uMax * _kt / _r;
}
        
double MotorModel::maxVelocity()
{
    return _uMax / _ke;
}
        
double MotorModel::maxTorque(double velocity)
{
    if (velocity > 0.0) {
        return (_uMax - _ke*velocity)*_kt/_r;
    } else {
        return maxTorque();
    }
}
double MotorModel::minTorque(double velocity)
{
    if (velocity > 0.0) {
        return -maxTorque();
    } else {
        return (-_uMax - _ke*velocity)*_kt/_r;
    }
}
        
double MotorModel::voltage(double velocity, double torque)
{
    return torque * _r / _kt + velocity * _ke;
}
Eigen::VectorXd MotorModel::voltage(
    const Eigen::VectorXd& velocity, const Eigen::VectorXd& torque)
{
    Eigen::VectorXd volt(velocity.size());
    for (size_t i=0;i<(size_t)volt.size();i++) {
        volt(i) = voltage(velocity(i), torque(i));
    }

    return volt;
}

}

