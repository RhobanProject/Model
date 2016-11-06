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
        const Eigen::VectorXd getParameters() const;
        void setParameters(const Eigen::VectorXd& params);

        /**
         * Return internal joint inertia
         */
        double getInertia() const;
        
        /**
         * Compute the torque applied on 
         * the joint by the mechanical friction
         * given current joint position and velocity.
         */
        double frictionTorque(double vel) const;

        /**
         * Compute the torque applied on 
         * the joint by the control algorithm
         * given current joint target position, 
         * current position and velocity.
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
         * Return current delayed goal
         */
        double getDelayedGoal() const;

        /**
         * Return current backlash hidden state
         */
        bool getBacklashStateEnabled() const;
        double getBacklashStatePos() const;
        double getBacklashStateVel() const;

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
         * external applied torque is given.
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
         * Current integrated time in seconds
         * and target goal history (for lag
         * implementation)
         */
        double _goalTime;
        std::queue<std::pair<double, double>> _goalHistory;

        /**
         * Backlash hidden relative 
         * enable and position state
         */
        bool _stateBacklashIsEnabled;
        double _stateBacklashPosition;
        double _stateBacklashVelocity;

        /**
         * Model parameters
         */
        //Friction Stribeck transtition velocity
        double _paramFrictionVelLimit;
        //Friction and inertia internal parameters
        double _paramInertiaIn;
        double _paramFrictionViscousIn;
        double _paramFrictionBreakIn;
        double _paramFrictionCoulombIn;
        //Friction and inertia external parameters
        double _paramInertiaOut;
        double _paramFrictionViscousOut;
        double _paramFrictionBreakOut;
        double _paramFrictionCoulombOut;
        //Electric motor parameters
        double _paramElectricVoltage;
        double _paramElectricKe;
        double _paramElectricResistance;
        //Control parameters
        double _paramControlGainP;
        double _paramControlDiscretization;
        //All inclusive time lag
        double _paramControlLag;
        //Backlash hysteresis parameters
        double _paramBacklashThresholdDeactivation;
        double _paramBacklashThresholdActivation;
        double _paramBacklashRangeMax;

        /**
         * Compute the friction force for given
         * velocity and optional given torque.
         * Internal and/or externzl gearbox friction
         * is used whenether isInFriction and isOutFriction
         * are set.
         */
        double computeFrictionTorque(
            double vel, double* torque, 
            bool isInFriction, bool isOutFriction) const;

        /**
         * Compute the control torque 
         * (without backlash model) from given
         * current position and velocity
         */
        double computeControlTorque(
            double pos, double vel) const;
};

}

#endif

