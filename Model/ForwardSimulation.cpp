#include "Model/ForwardSimulation.hpp"
#include "Utils/RungeKutta4.hpp"

namespace Leph {

ForwardSimulation::ForwardSimulation(Model& model) :
    _model(&model),
    _jointModels(),
    _positions(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _velocities(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _actives(Eigen::VectorXi::Zero(model.sizeDOF())),
    _goals(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _accelerations(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _outputTorques(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _frictionTorques(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _controlTorques(Eigen::VectorXd::Zero(_model->sizeDOF())),
    _inputTorques(Eigen::VectorXd::Zero(_model->sizeDOF()))
{
    //Init joint models
    for (size_t i=0;i<_model->sizeDOF();i++) {
        std::string name = _model->getDOFName(i);
        //Special base DOF case set 
        //as Free and activated
        if (name.find("base_") != std::string::npos) {
            _actives(i) = 1;
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
const Eigen::VectorXi& ForwardSimulation::actives() const
{
    return _actives;
}
Eigen::VectorXi& ForwardSimulation::actives()
{
    return _actives;
}
const Eigen::VectorXd& ForwardSimulation::goals() const
{
    return _goals;
}
Eigen::VectorXd& ForwardSimulation::goals()
{
    return _goals;
}
const Eigen::VectorXd& ForwardSimulation::outputTorques() const
{
    return _outputTorques;
}
const Eigen::VectorXd& ForwardSimulation::frictionTorques() const
{
    return _frictionTorques;
}
const Eigen::VectorXd& ForwardSimulation::controlTorques() const
{
    return _controlTorques;
}
const Eigen::VectorXd& ForwardSimulation::inputTorques() const
{
    return _inputTorques;
}
const Eigen::VectorXd& ForwardSimulation::accelerations() const
{
    return _accelerations;
}

void ForwardSimulation::update(double dt,
    RBDL::ConstraintSet* constraints)
{
    //Save last computed velocities and accelerations
    size_t size = _model->sizeDOF();
    Eigen::VectorXd lastVelocities = _velocities;
    Eigen::VectorXd lastAccelerations = _accelerations;

    //Update joint model state
    for (size_t i=0;i<size;i++) {
        _jointModels[i].updateState(
            dt, _goals(i), _positions(i), _velocities(i));
    }

    //Compute and return the generalized state
    //derivative from given state
    //(first vector part is position, second part is
    //velocity)
    //Call model forward dynamics
    auto differential = 
        [this, &lastVelocities, constraints]
        (const Eigen::VectorXd& state) -> Eigen::VectorXd 
    {
        size_t size = this->_model->sizeDOF();
        //Compute joint friction and control torque
        for (size_t i=0;i<size;i++) {
            if (this->_actives(i) != 0) {
                //Compute non static control and torque friction
                this->_frictionTorques(i) = this->_jointModels[i]
                    .frictionTorque(state(i), state(size+i));
                this->_controlTorques(i) = this->_jointModels[i]
                    .controlTorque(state(i), state(size+i));
                //If a joint has just been activated 
                //in the last iteration, its velocity is zero.
                //The friction torque sign could not be computed using
                //the velocity. For this initial iteration, the current 
                //friction torque sign is set to counter the applied
                //external torque (same as for disable to enable condition) 
                //of last iteration as if it was simulated and not moving.
                if (lastVelocities(i) == 0.0) { 
                    this->_frictionTorques(i) = 
                        (this->_inputTorques(i)+this->_controlTorques(i) 
                            > 0.0 ? -1.0 : 1.0)
                        * fabs(this->_frictionTorques(i));
                }
                //Sum applied torque on the joint
                this->_outputTorques(i) = 
                    this->_frictionTorques(i) + 
                    this->_controlTorques(i);
            } else {
                //No applied torque (not used) if the joint is disabled
                this->_outputTorques(i) = 0.0;
            }
        }
        //Compute partial (with fixed DOF 
        //for static friction) Forward Dynamics
        if (constraints == nullptr) {
            this->_accelerations = this->_model->forwardDynamicsPartial(
                state.segment(0, size),
                state.segment(size, size),
                this->_outputTorques,
                this->_actives, 
                RBDLMath::LinearSolverFullPivHouseholderQR);
        } else {
            this->_accelerations = 
                this->_model->forwardDynamicsContactsPartial(
                *constraints,
                state.segment(0, size),
                state.segment(size, size),
                this->_outputTorques,
                this->_actives,
                RBDLMath::LinearSolverFullPivHouseholderQR);
        }
        //Build generalized state derivative
        Eigen::VectorXd diff(2*size);
        diff.segment(0, size) = state.segment(size, size);
        diff.segment(size, size) = this->_accelerations;
        return diff;
    };
    
    //Build generalized state
    Eigen::VectorXd state(2*size);
    state.segment(0, size) = _positions;
    state.segment(size, size) = _velocities;

    //Compute next state with 
    //Euler integration.
    //(Runge-Kutta integration cannot be used
    //since the differential is not continuous
    //and cannot be averaged)
    /*
    Eigen::VectorXd nextState = RungeKutta4Integration(
        state, dt, differential);
    */
    Eigen::VectorXd nextState = state + dt*differential(state);

    //Retrieve data
    _positions = nextState.segment(0, size);
    _velocities = nextState.segment(size, size);
    //Assign model position state
    _model->setDOFVect(_positions);
    
    //Recompute with Inverse Dynamics 
    //applied torque on each DOF
    //(minus because InverseDynamics compute the 
    //needed torque to produce given acceleration).
    if (constraints == nullptr) {
        _inputTorques = - _model->inverseDynamics(
            _velocities, _accelerations);
    } else {
        _inputTorques = - _model->inverseDynamicsContacts(
            *constraints, _positions, _velocities, _accelerations);
    }

    //Recompute all friction 
    //and control torque
    for (size_t i=0;i<size;i++) {
        _frictionTorques(i) = _jointModels[i]
            .frictionTorque(_positions(i), _velocities(i));
        _controlTorques(i) = _jointModels[i]
            .controlTorque(_positions(i), _velocities(i));
    }

    //Handle joint activation and static friction
    bool isOneActivation = false;
    bool isOneDeactivation = false;
    for (size_t i=0;i<size;i++) {
        //Compute current friction torque 
        //in case of static state
        double staticFrictionCurrent = 
            fabs(_inputTorques(i) + _controlTorques(i));
        //Compute friction torque limit at zero velocity (Coulomb cone).
        //1.01 prevent false numerical activation of joint
        double staticFrictionLimit = 
            1.01*fabs(_jointModels[i].frictionTorque(_positions(i), 0.0));
        //Active to disabled
        if (
            _jointModels[i].getType() != JointModel::JointFree &&
            _actives(i) != 0 &&
            ((_velocities(i) < 0.0 && lastVelocities(i) > 0.0) ||
            (_velocities(i) > 0.0 && lastVelocities(i) < 0.0))
        ) {
            //Compute with forwardDynamics the acceleration
            //of the next iteration in order to check if
            //its sign will changed
            Eigen::VectorXd tmpNextAcceleration;
            if (constraints == nullptr) {
                //No constraint
                tmpNextAcceleration = _model->forwardDynamicsPartial(
                    _positions,
                    _velocities,
                    _frictionTorques + _controlTorques,
                    _actives,
                    RBDLMath::LinearSolverFullPivHouseholderQR);
            } else {
                //Constraints
                //(save and restore computed forces)
                Eigen::VectorXd tmpSaveForce = constraints->force;
                tmpNextAcceleration = _model->forwardDynamicsContactsPartial(
                    *constraints,
                    _positions,
                    _velocities,
                    _frictionTorques + _controlTorques,
                    _actives,
                    RBDLMath::LinearSolverFullPivHouseholderQR);
                constraints->force = tmpSaveForce;
            }
            //Joint are really disabled when the friction force
            //causes the velocity sign to change and also when
            //current or next acceleration sign is changed
            if (
                (_accelerations(i) < 0.0 && tmpNextAcceleration(i) > 0.0) ||
                (_accelerations(i) > 0.0 && tmpNextAcceleration(i) < 0.0) ||
                (_accelerations(i) < 0.0 && lastAccelerations(i) > 0.0) ||
                (_accelerations(i) > 0.0 && lastAccelerations(i) < 0.0)
            ) {
                //Disable the joint when the friction 
                //causes velocity sign change to goes to 
                //zero velocity
                _velocities(i) = 0.0;
                _accelerations(i) = 0.0;
                _actives(i) = 0;
                isOneDeactivation = true;
            }
        //Disabled to active
        } else if (
            _jointModels[i].getType() != JointModel::JointFree &&
            _actives(i) == 0 &&
            !isOneActivation &&
            staticFrictionCurrent > staticFrictionLimit
        ) {
            //Enable stopped joint due to static friction
            //when applied high enough torque.
            _actives(i) = 1;
            //At must one joint as activated at each iteration
            //to avoid numerical instalibity of multiple
            //newly activated joints
            isOneActivation = true;
        }
    }
    //When a DOF is disabled, the velocity is set
    //to zero and if constraints are not empty,
    //the new velocity may break the constraints.
    //An impulsion need to be applied to enforce
    //the constraints with the new velocity and model.
    if (isOneDeactivation && constraints != nullptr) {
        _velocities = _model->impulseContactsPartial(
            *constraints,
            _positions,
            _velocities,
            _actives,
            RBDLMath::LinearSolverFullPivHouseholderQR);
    }

    //Check numerical validity
    if (
        !_positions.allFinite() ||
        !_velocities.allFinite() ||
        !_accelerations.allFinite() ||
        !_goals.allFinite() ||
        !_inputTorques.allFinite() ||
        !_frictionTorques.allFinite() ||
        !_controlTorques.allFinite() ||
        !_outputTorques.allFinite()
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

