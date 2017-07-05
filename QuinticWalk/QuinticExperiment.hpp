#ifndef LEPH_QUINTICWALK_HPP
#define LEPH_QUINTICWALK_HPP

#include <Eigen/Dense>
#include "Model/HumanoidFixedModel.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Spline/SplineContainer.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h" 

namespace Leph {

/**
 * QuinticWalk
 *
 * Walk generator based on polynomial 
 * degree 5th splines in cartesian space.
 * The walk is parametrized using foot/trunk
 * position/orientation in support foot frame
 * allowing to compute analytical velocity and
 * acceleration trajectories
 */
class QuinticWalk
{
    public:

        /**
         * QuinticWalk parameters
         */
        typedef Eigen::VectorXd Parameters;

        /**
         * Initialization with 
         * default parameters
         */
        QuinticWalk();

        /**
         * Set walk parameters and re-generate 
         * cartesian trajectories
         */
        void setParameters(const Parameters& params);

        /**
         * Return currently used parameters
         */
        const Parameters getParameters() const;

        /**
         * Access to computed Trajectories
         */
        const Trajectories& getTrajectories() const;

        /**
         * Compute degrees of freedom target position
         * with given Humanoid Model and assign DOF position
         * at given phase between 0 and 1.
         * False is returned if Inverse Kinematics fails.
         */
        bool computePose(
            HumanoidFixedModel& model, double phase) const;

        /**
         * Return updated given phase (between 0 and 1)
         * accordingly with given time step and current
         * parameters (walk frequency)
         */
        double updatePhase(double phase, double dt) const;

        /**
         * Return trajectory time from given phase
         * between 0 and 1
         */
        double phaseToTime(double phase) const;

        /**
         * Return default walk parameters
         */
        static Parameters defaultParameters();

        /**
         * Re-generate cartesian foot/trunk pose
         * trajectories from current walk parameters
         */
        Trajectories generateTrajectories() const;

    private:

        /**
         * Current walk parameters
         */
        Parameters _params;

        /**
         * Cached cycle Trajectories associated
         * with current walk parameters
         */
        Trajectories _trajs;
};

}

#endif

