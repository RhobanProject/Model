#include "Model/ForwardSimulation.hpp"

namespace Leph {

MotorModel::MotorModel()
{
    //DXL MX64 configuration
    //(V)
    _uMax = 12.0;
    //(V.s/rad)
    _ke = 1.399;
    //(N.m/A)
    _kt = _ke;
    //(ohm)
    _r = 4.07;

    //Position P gain
    _positionControlGain = 50.0;
}
    
double MotorModel::computeTorque(
    double goal, double pos, double vel)
{
    double torque = _positionControlGain*(goal - pos);

    //Bound torque
    double tauMax = _uMax/_ke;
    if (vel > 0.0) {
        double tmp = (_uMax - _ke*vel)*_kt/_r;
        if (torque < -tauMax) torque = -tauMax;
        if (torque > tmp) torque = tmp;
    } else {
        double tmp = (-_uMax - _ke*vel)*_kt/_r;
        if (torque < tmp) torque = tmp;
        if (torque > tauMax) torque = tauMax;
    }

    return torque;
}
        
void MotorModel::boundState(double& pos, double& vel) const
{
    //Bound position range
    if (pos < -M_PI) pos = -M_PI;
    if (pos > M_PI) pos = M_PI;
    //Bound velocity range
    double velMax = _uMax/_ke;
    if (vel < -velMax) vel = -velMax;
    if (vel > velMax) vel = velMax;
}
        
ForwardSimulation::ForwardSimulation(Model& model) :
    _model(&model),
    _position(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _velocity(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _goal(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _torque(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _acceleration(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _motors()
{
    //Init motors model
    for (size_t i=0;i<_model->sizeDOF();i++) {
        _motors.push_back(MotorModel());
    }
}

Eigen::VectorXd& ForwardSimulation::position()
{
    return _position;
}
Eigen::VectorXd& ForwardSimulation::velocity()
{
    return _velocity;
}
Eigen::VectorXd& ForwardSimulation::goal()
{
    return _goal;
}
Eigen::VectorXd& ForwardSimulation::torque()
{
    return _torque;
}
Eigen::VectorXd& ForwardSimulation::acceleration()
{
    return _acceleration;
}
        
void ForwardSimulation::update(double dt)
{
    size_t size = _model->sizeDOF();
    
    //Update DOF torque from goal
    for (size_t i=0;i<size;i++) {
        _torque(i) = _motors[i].computeTorque(
            _goal(i), _position(i), _velocity(i));
    }

    //Build generalized state
    Eigen::VectorXd state(2*size);
    state.segment(0, size) = _position;
    state.segment(size, size) = _velocity;

    //Compute next state
    Eigen::VectorXd nextState = RungeKutta4(dt, state);
    //Retrieve data
    _position = nextState.segment(0, size);
    _velocity = nextState.segment(size, size);
    
    //Bound state
    for (size_t i=0;i<size;i++) {
        _motors[i].boundState(
            _position(i), _velocity(i));
    }
}
        
Eigen::VectorXd ForwardSimulation::generalizedModelDiff(
    const Eigen::VectorXd& state)
{
    size_t size = _model->sizeDOF();
    _acceleration = _model->forwardDynamics(
        state.segment(0, size),
        state.segment(size, size),
        _torque
    );

    Eigen::VectorXd diff(2*size);
    diff.segment(0, size) = state.segment(size, size);
    diff.segment(size, size) = _acceleration;

    return diff;
}
        
Eigen::VectorXd ForwardSimulation::RungeKutta4(
    double dt, const Eigen::VectorXd& state)
{
    Eigen::VectorXd k1 = generalizedModelDiff(state);
    Eigen::VectorXd k2 = generalizedModelDiff(state + 0.5*dt*k1);
    Eigen::VectorXd k3 = generalizedModelDiff(state + 0.5*dt*k2);
    Eigen::VectorXd k4 = generalizedModelDiff(state + dt*k3);

    return state + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

}

