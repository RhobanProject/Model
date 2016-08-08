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
            //(no parameters)
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
         * Compute the torque applied on 
         * the joint by the mechanical friction
         * given current joint position and velocity
         */
        double frictionTorque(
            double pos, double vel);

        /**
         * Compute the torque applied on 
         * the joint by the control algorithm
         * given current joint target position, 
         * current position and velocity.
         * The timestep update is also given for 
         * implementation of the lag.
         */
        double controlTorque(
            double dt, double goal, 
            double pos, double vel);

        /**
         * Optionnaly update given current joint
         * position and velocity to ensure constraints
         */
        void boundState(double& pos, double& vel);

        /**
         * Read access to joint state
         */
        const Eigen::VectorXd& getState() const;

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
         * Joint optionaly used state
         * for control method
         */
        Eigen::VectorXd _state;

        /**
         * Current time counter in seconds
         * since start of simutation
         */
        double _timeCounter;

        /**
         * Hold last received goal
         * associated with a timestamp
         * (implement hardware lag)
         */
        std::deque<std::pair<double, double>> _history;
};

}

#endif

