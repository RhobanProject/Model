#ifndef LEPH_FORWARDHUMANOIDSIMULATION_HPP
#define LEPH_FORWARDHUMANOIDSIMULATION_HPP

#include <map>
#include <string>
#include <Eigen/Dense>
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Plot/Plot.hpp"

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
         * Initialization with robot type.
         * If inertia data and name are not empty,
         * given inertia override default model data.
         * If geometry data and name are not empty,
         * given geometry override default model data.
         */
        HumanoidSimulation(
            RobotType type,
            const Eigen::MatrixXd& inertiaData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& inertiaName = {},
            const Eigen::MatrixXd& geometryData = Eigen::MatrixXd(),
            const std::map<std::string, size_t>& geometryName = {});

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
         * Set the floating base (trunk) orientation
         * to set the lowest foot (or given foot) flat 
         * on the ground
         */
        void putOnGround();
        void putOnGround(
            HumanoidFixedModel::SupportFoot foot);

        /**
         * Set the floating base translation (trunk)
         * to set the lowest foot (or given foot) 
         * at given position
         */
        void putFootAt(double x, double y);
        void putFootAt(double x, double y, 
             HumanoidFixedModel::SupportFoot foot);

        /**
         * Internal state access
         */
        const Eigen::VectorXd& positions() const;
        Eigen::VectorXd& positions();
        const Eigen::VectorXd& velocities() const;
        Eigen::VectorXd& velocities();
        const Eigen::VectorXd& goals() const;
        Eigen::VectorXd& goals();
        const Eigen::VectorXd& outputTorques() const;
        const Eigen::VectorXd& frictionTorques() const;
        const Eigen::VectorXd& controlTorques() const;
        const Eigen::VectorXd& inputTorques() const;
        const Eigen::VectorXd& accelerations() const;

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
         * Return the vertical force appling 
         * on given cleat name
         */
        double getCleatForce(const std::string& name) const;

        /**
         * Assign given joints parameters to all 
         * internal joint models
         */
        void setJointModelParameters(const Eigen::VectorXd& params);

        /**
         * Run the forward simulation by one step
         * of given time duration. Handle contact
         * and collision constraints.
         */
        void update(double dt);

        /**
         * Display on standart output deguging
         * info on cleats status
         */
        void printCleatsStatus(bool verbose = true);
        void printCleatsStatus(Plot& plot);

    private:

        /**
         * Structure for cleat constraint state
         */
        struct CleatState {
            //Model frame name
            std::string frame;
            //Cleat number between
            //1 and 4 typicaly
            int number;
            //RBDL body id
            size_t bodyId;
            //Is left or right leg
            bool isLeft;
            //Is currently in active constraints set
            bool isActive;
            //Is currently on the ground
            bool isContact;
            //If active, index of Z in constraints set
            size_t index;
            //Vertical Z force.
            //Non zero if the active
            double force;
            //Temporary cleat height
            //for constraints management
            double height;
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

        /**
         * Assign current cleat active state to
         * given combination.
         * The set size is the number of constraints
         * and the index is the number of activated cleat.
         * Only given foot is assign.
         * True is returned when only foot diagonal cleats
         * are activated.
         */
        bool setActiveCombination(
            const std::vector<size_t>& set, bool isLeft);

        /**
         * Try out all possible combination of enabled
         * constraints for left or right foot (depending on
         * is Left is true) to find a valid active set.
         * The given dt is used to simulate the outcome
         * one step ahead.
         * Valid active constraint set are assign 
         * (buildConstraints) and constraints are enforced.
         */
        void findActiveConstraints(double dt, bool isLeft);
};

}

#endif

