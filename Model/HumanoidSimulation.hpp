#ifndef LEPH_FORWARDHUMANOIDSIMULATION_HPP
#define LEPH_FORWARDHUMANOIDSIMULATION_HPP

#include <map>
#include <string>
#include <Eigen/Dense>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"

namespace Leph {

/**
 * HumanoidSimulation
 *
 * Special implementation using generic 
 * ForwardSimulation for Humanoid models
 * (Sigmaban, Grosban).
 * Handle the foot cleats contact and
 * simulation valid state.
 */
class HumanoidSimulation
{
    public:

        /**
         * Initialization with robot type
         */
        HumanoidSimulation(RobotType type);

        /**
         * Deallocation of ConstraintSet
         */
        ~HumanoidSimulation();

        /**
         * Access to internal simulated model
         */
        const HumanoidModel& model() const;
        HumanoidModel& model();

        /**
         * Set the current model with the
         * lowest foot flat on the ground
         */
        void putOnGround();

        /**
         * Set the current lowest foot
         * at given position
         */
        void putFootAt(double x, double y);

        /**
         * Internal state access
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
         * Set a target goal or position to given DOF name
         */
        void setGoal(const std::string& name, double pos);
        void setPos(const std::string& name, double pos);
        void setVel(const std::string& name, double pos);

        /**
         * Return state for given DOF name
         */
        double getGoal(const std::string& name) const;
        double getPos(const std::string& name) const;
        double getVel(const std::string& name) const;

        /**
         * Direct access to internal joint model
         */
        const JointModel& jointModel(const std::string& name) const;
        JointModel& jointModel(const std::string& name);

        /**
         * Assign given joints parameters to all 
         * internal joint models
         */
        void setJointModelParameters(const Eigen::VectorXd& params);

        /**
         * Return the computed force acting on given 
         * foot cleat frame name
         */
        const Eigen::VectorXd& getCleatForce(
            const std::string& name) const;

        /**
         * Return the total vertical weight
         * and vertical weight ratio computed 
         * on cleats in kilogramms
         */
        double getWeightSum() const;
        double getWeightLeftRatio() const;
        double getWeightRightRatio() const;

        /**
         * Run the forward simulation by one step
         * of given time duration. Handle contact
         * and collision constraints.
         */
        void update(double dt);

    private:

        /**
         * Structure for fot cleat constraint state
         */
        struct CleatState {
            std::string frame;
            bool isLeft;
            bool isEnabled;
            size_t index;
            Eigen::VectorXd force;
        };

        /**
         * Robot Humanoid model
         */
        HumanoidModel _model;

        /**
         * Internal ForwardSimulation
         */
        ForwardSimulation _simulation;

        /**
         * RBDL current constraint set instance
         */
        RBDL::ConstraintSet* _constraints;

        /**
         * All fot cleats current state container
         * indexed by cleat name
         */
        std::map<std::string, CleatState> _cleats;

        /**
         * Add and configure a cleat with given name
         */
        void addCleat(const std::string& frame);

        /**
         * Rebuild the RBDL constraint set
         * from foot cleats state
         */
        void buildConstraints();
};

}

#endif

