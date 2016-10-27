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
            //(no parameter)
            JointFree,
            //Friction and control torque
            JointActuated,
        };

        /**
         * Initialization with 
         * jont type and name
         */
        JointModel(
            JointModelType type = JointActuated, 
            const std::string& name = "");

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
         * given current joint position and velocity.
         * If isBacklash is false, the backlask model
         * is disabled.
         */
        double frictionTorque(
            double vel, bool isBacklash = true) const;

        /**
         * Compute the torque applied on 
         * the joint by the control algorithm
         * given current joint target position, 
         * current position and velocity.
         */
        double controlTorque(double pos, double vel) const;

        /**
         * Compute and return for 
         * actuated joint the minimum and maximum
         * bounds for control torque.
         */
        double controlMaxTorque(double vel) const;
        double controlMinTorque(double vel) const;

        /**
         * Update current joint friction and
         * control state from given simulation
         * step, target goal position and current state
         */
        void updateState(
            double dt, double goal, double pos, double vel);

        /**
         * Return current delayed goal
         */
        double getDelayedGoal() const;

        /**
         * Return current backlash hidden state
         */
        double getBacklashState() const;

        /**
         * Optionnaly update given current joint
         * position and velocity to ensure constraints
         */
        void boundState(double& pos, double& vel);

        /**
         * Compute the actual torque seen by the
         * control motor and return ratio between
         * this torque and control maximum torque bound.
         * The returnded value is always positive or zero.
         * If between 0 and 1, the current torque can be
         * provided by the motor.
         * Current joint velocity, acceleration and
         * external apllied torque is given.
         * Backlash model is not used.
         */
        double ratioMaxControlTorque(
            double vel, double acc, double torque) const;

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
         * Current integrated time in seconds
         * and target goal history (for lag
         * implementation)
         */
        double _goalTime;
        std::queue<std::pair<double, double>> _goalHistory;

        /**
         * Backlash hidden relative 
         * position state
         */
        double _backlashPosition;
};

}

#endif

