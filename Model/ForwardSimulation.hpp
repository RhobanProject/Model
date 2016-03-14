#ifndef LEPH_FORWARDSIMULATION_H
#define LEPH_FORWARDSIMULATION_H

#include <Eigen/Dense>
#include <vector>
#include "Model/Model.hpp"

namespace Leph {

/**
 * MotorModel
 *
 * Class for servo-motors
 * model used to compute output
 * torque from goal and state.
 * Use a simple Proportional 
 * Integral position controller
 */
class MotorModel
{
    public:

        /**
         * Initialization with
         * Dynamixel MX64 configuration
         */
        MotorModel();

        /**
         * Compute the motor output torque from current
         * goal position, realposition and velocity
         */
        double computeTorque(
            double goal, double pos, double vel);

        /**
         * Enforce motor constraint and update 
         * given position and velocity state
         */
        void boundState(double& pos, double& vel) const;

    private:

        /**
         * Motor configuration
         */
        double _uMax;
        double _ke;
        double _kt;
        double _r;

        /**
         * Control configuration
         */
        double _positionControlGain;
};

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
         * Initialization with Model instance
         */
        ForwardSimulation(Model& model);

        /**
         * Access to current state
         */
        Eigen::VectorXd& position();
        Eigen::VectorXd& velocity();
        Eigen::VectorXd& goal();
        Eigen::VectorXd& torque();
        Eigen::VectorXd& acceleration();

        /**
         * Update the position/velocity state
         * from current goal position over
         * the given dt time step
         */
        void update(double dt);

    private:

        /**
         * Model pointer instance
         */
        Model* _model;

        /**
         * Degrees of freedom position
         * and velocity state
         */
        Eigen::VectorXd _position;
        Eigen::VectorXd _velocity;

        /**
         * Degrees of freedom current
         * goal position
         */
        Eigen::VectorXd _goal;

        /**
         * Last computed torques and 
         * accelerations for each DOF
         */
        Eigen::VectorXd _torque;
        Eigen::VectorXd _acceleration;

        /**
         * Container of Motor Model for each
         * degrees of freedom
         */
        std::vector<MotorModel> _motors;

        /**
         * Compute and return the generalized state
         * derivative from given state
         * (first vector part is position, second part is
         * velocity)
         * Call model forward dynamics
         */
        Eigen::VectorXd generalizedModelDiff(
            const Eigen::VectorXd& state);

        /**
         * Integrate the system using Runge Kutta 4
         * method along dt given time and return next state
         */
        Eigen::VectorXd RungeKutta4(
            double dt, const Eigen::VectorXd& state); 
};

}

#endif

