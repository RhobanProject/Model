#ifndef LEPH_JOINTMODEL_HPP
#define LEPH_JOINTMODEL_HPP

#include <string>
#include <Eigen/Dense>
#include <queue>

namespace Leph {

/**
 * JointModel
 *
 * Mechanical and control model of
 * a joint for its use in Forward Simulation.
 * Multiple joint types are implemented.
 */
class JointModel
{
    public:

        /**
         * Type of Joint model
         */
        enum JointModelType {
            //No friction, no control torque
            //(no parameter or state)
            JointFree,
            //Friction and control torque
            JointActuated,
        };

        /**
         * Initialization with 
         * jont type and name
         */
        JointModel(
            JointModelType type, 
            const std::string& name);

        /**
         * Return the joint type
         */
        JointModelType getType() const;

        /**
         * Return joint textual name
         */
        const std::string& getName() const;

        /**
         * Get and set internal 
         * model parameters
         */
        const Eigen::VectorXd& getParameters() const;
        void setParameters(const Eigen::VectorXd& params);
        
        /**
         * Get and set internal joint state
         */
        const Eigen::VectorXd& getStates() const;
        void setStates(const Eigen::VectorXd& states);

        /**
         * Compute the torque applied on 
         * the joint by the mechanical friction
         * given current joint position and velocity.
         * (state less const)
         */
        double frictionTorque(double pos, double vel) const;

        /**
         * Compute the torque applied on 
         * the joint by the control algorithm
         * given current joint target position, 
         * current position and velocity.
         * (state less const)
         */
        double controlTorque(double pos, double vel) const;

        /**
         * Update current joint friction and
         * control state from given simulation
         * step, target goal position and current state
         */
        void updateState(
            double dt, double goal, double pos, double vel);

        /**
         * Optionnaly update given current joint
         * position and velocity to ensure constraints
         */
        void boundState(double& pos, double& vel);

    private:

        /**
         * Joint model current type
         */
        JointModelType _type;

        /**
         * Joint textual name
         */
        std::string _name;

        /**
         * Model parameters
         */
        Eigen::VectorXd _parameters;

        /**
         * Joint used state
         * for control method
         */
        Eigen::VectorXd _states;

        /**
         * Is updateState have already be
         * called once
         */
        bool _isInitialized;
};

}

#endif

