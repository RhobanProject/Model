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
         * and dynamics data. It is not updated
         * automaticaly.
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
         * Access to current state
         */
        const Eigen::VectorXd& positions() const;
        Eigen::VectorXd& positions();
        const Eigen::VectorXd& velocities() const;
        Eigen::VectorXd& velocities();
        const Eigen::VectorXd& goals() const;
        Eigen::VectorXd& goals();
        const Eigen::VectorXd& jointTorques() const;
        Eigen::VectorXd& jointTorques();
        const Eigen::VectorXd& accelerations() const;
        Eigen::VectorXd& accelerations();

        /**
         * Update the position/velocity state
         * from current goal position over
         * the given dt time step.
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
         * Degrees of freedom current
         * goal position
         */
        Eigen::VectorXd _goals;

        /**
         * Last computed accelerations and last 
         * generated output torque for each DOF
         */
        Eigen::VectorXd _jointTorques;
        Eigen::VectorXd _accelerations;
};

}

#endif

