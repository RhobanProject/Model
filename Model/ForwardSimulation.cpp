#include "Model/ForwardSimulation.hpp"
#include "Utils/RungeKutta4.hpp"

namespace Leph {

ForwardSimulation::ForwardSimulation(Model& model) :
    _model(&model),
    _jointModels(),
    _positions(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _velocities(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _goals(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _jointTorques(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _accelerations(Eigen::VectorXd::Zero(_model->sizeDOF()))
{
    //Init joint models
    for (size_t i=0;i<_model->sizeDOF();i++) {
        std::string name = _model->getDOFName(i);
        //Spacial base DOF ate set as free
        if (name.find("base_") != std::string::npos) {
            _jointModels.push_back(JointModel(
                JointModel::JointFree, name));
        } else {
            _jointModels.push_back(JointModel(
                JointModel::JointActuated, name));
        }
    }
    //Load state
    _positions = _model->getDOFVect();
    _goals = _model->getDOFVect();
}
        
const JointModel& ForwardSimulation::jointModel(size_t index) const
{
    if (index >= _jointModels.size()) {
        throw std::logic_error("ForwardSimulation invalid index");
    }
    return _jointModels[index];
}
JointModel& ForwardSimulation::jointModel(size_t index)
{
    if (index >= _jointModels.size()) {
        throw std::logic_error("ForwardSimulation invalid index");
    }
    return _jointModels[index];
}
const JointModel& ForwardSimulation::jointModel(const std::string& name) const
{
    return jointModel(_model->getDOFIndex(name));
}
JointModel& ForwardSimulation::jointModel(const std::string& name)
{
    return jointModel(_model->getDOFIndex(name));
}
        
void ForwardSimulation::setJointModelParameters(
    const Eigen::VectorXd& params)
{
    for (size_t i=0;i<_jointModels.size();i++) {
        if (_jointModels[i].getType() == JointModel::JointActuated) {
            _jointModels[i].setParameters(params);
        }
    }
}

const Eigen::VectorXd& ForwardSimulation::positions() const
{
    return _positions;
}
Eigen::VectorXd& ForwardSimulation::positions()
{
    return _positions;
}
const Eigen::VectorXd& ForwardSimulation::velocities() const
{
    return _velocities;
}
Eigen::VectorXd& ForwardSimulation::velocities()
{
    return _velocities;
}
const Eigen::VectorXd& ForwardSimulation::goals() const
{
    return _goals;
}
Eigen::VectorXd& ForwardSimulation::goals()
{
    return _goals;
}
const Eigen::VectorXd& ForwardSimulation::jointTorques() const
{
    return _jointTorques;
}
Eigen::VectorXd& ForwardSimulation::jointTorques()
{
    return _jointTorques;
}
const Eigen::VectorXd& ForwardSimulation::accelerations() const
{
    return _accelerations;
}
Eigen::VectorXd& ForwardSimulation::accelerations()
{
    return _accelerations;
}

void ForwardSimulation::update(double dt,
    RBDL::ConstraintSet* constraints)
{
    size_t size = _model->sizeDOF();

    //Update DOF output torque from goal
    //and current state
    for (size_t i=0;i<size;i++) {
        _jointTorques(i) = 0.0;
        _jointTorques(i) += _jointModels[i].frictionTorque(
            _positions(i), _velocities(i));
        _jointTorques(i) += _jointModels[i].controlTorque(
            dt, _goals(i), _positions(i), _velocities(i)); 
    }

    //Build generalized state
    Eigen::VectorXd state(2*size);
    state.segment(0, size) = _positions;
    state.segment(size, size) = _velocities;

    //Compute and return the generalized state
    //derivative from given state
    //(first vector part is position, second part is
    //velocity)
    //Call model forward dynamics
    auto differential = 
        [this, constraints](const Eigen::VectorXd& state) -> Eigen::VectorXd 
    {
        size_t size = this->_model->sizeDOF();
        if (constraints == nullptr) {
            this->_accelerations = this->_model->forwardDynamics(
                state.segment(0, size),
                state.segment(size, size),
                this->_jointTorques);
        } else {
            this->_accelerations = this->_model->forwardDynamicsContacts(
                *constraints,
                state.segment(0, size),
                state.segment(size, size),
                this->_jointTorques);
        }

        Eigen::VectorXd diff(2*size);
        diff.segment(0, size) = state.segment(size, size);
        diff.segment(size, size) = this->_accelerations;

        return diff;
    };

    //Compute next state
    Eigen::VectorXd nextState = RungeKutta4Integration(
        state, dt, differential);

    //Retrieve data
    _positions = nextState.segment(0, size);
    _velocities = nextState.segment(size, size);

    //Check numerical validity
    if (
        !_positions.allFinite() ||
        !_velocities.allFinite() ||
        !_accelerations.allFinite() ||
        !_goals.allFinite() ||
        !_jointTorques.allFinite()
    ) {
        throw std::runtime_error(
            "ForwardSimulation numerical instability");
    }
    
    //Bound joint state
    for (size_t i=0;i<size;i++) {
        _jointModels[i].boundState(_positions(i), _velocities(i));
    }
}

}

