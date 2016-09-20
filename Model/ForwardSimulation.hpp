#ifndef LEPH_FORWARDSIMULATION_H
#define LEPH_FORWARDSIMULATION_H

#include <vector>
#include <Eigen/Dense>
#include "Model/Model.hpp"
#include "Model/JointModel.hpp"

namespace Leph {

/**
 * ForwardSimulation
 *
 * Model simulation using forward
 * dynamics and classic Runge Kutta 
 * integration.
 */
class ForwardSimulation
{
    public:

        /**
         * Initialization with Model instance.
         * The given model is used for kinematcis
         * and dynamics data. 
         */
        ForwardSimulation(Model& model);

        /**
         * Access to given joint model by its
         * index or name
         */
        const JointModel& jointModel(size_t index) const;
        JointModel& jointModel(size_t index);
        const JointModel& jointModel(const std::string& name) const;
        JointModel& jointModel(const std::string& name);
        
        /**
         * Assign given joints parameters to all 
         * internal joint models
         */
        void setJointModelParameters(const Eigen::VectorXd& params);

        /**
         * Access to current state
         */
        const Eigen::VectorXd& positions() const;
        Eigen::VectorXd& positions();
        const Eigen::VectorXd& velocities() const;
        Eigen::VectorXd& velocities();
        const Eigen::VectorXi& actives() const;
        Eigen::VectorXi& actives();
        const Eigen::VectorXd& goals() const;
        Eigen::VectorXd& goals();
        const Eigen::VectorXd& outputTorques() const;
        const Eigen::VectorXd& frictionTorques() const;
        const Eigen::VectorXd& controlTorques() const;
        const Eigen::VectorXd& inputTorques() const;
        const Eigen::VectorXd& accelerations() const;

        /**
         * Update the position/velocity state
         * from current goal position over
         * the given dt time step.
         * Underlying model position is updated.
         * Optionnaly use given RBDL contact 
         * constraint set.
         */
        void update(double dt, 
            RBDL::ConstraintSet* constraints = nullptr);
        
    private:

        /**
         * Model pointer instance
         */
        Model* _model;

        /**
         * Joint model for all degrees of freedom.
         */
        std::vector<JointModel> _jointModels;

        /**
         * Degrees of freedom position
         * and velocity state
         */
        Eigen::VectorXd _positions;
        Eigen::VectorXd _velocities;
        
        /**
         * Associated degrees of freedom
         * are fixed if the actives vector
         * value is zero. Non zero means
         * non fixed.
         * (Used for static friction)
         */
        Eigen::VectorXi _actives;

        /**
         * Degrees of freedom current
         * goal position
         */
        Eigen::VectorXd _goals;

        /**
         * Last computed accelerations, last 
         * generated output torque (decomposed in 
         * friction and control) and computed
         * with inverse dynamics torques for each DOF
         */
        Eigen::VectorXd _accelerations;
        Eigen::VectorXd _outputTorques;
        Eigen::VectorXd _frictionTorques;
        Eigen::VectorXd _controlTorques;
        Eigen::VectorXd _inputTorques;
};

}

#endif

