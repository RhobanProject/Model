#ifndef LEPH_TRAJECTORYGENERATION_HPP
#define LEPH_TRAJECTORYGENERATION_HPP

#include <functional>
#include <Eigen/Dense>
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"

namespace Leph {

/**
 * TrajectoryGeneration
 *
 * Interface for Smooth Trajectory
 * generation through CMA-ES splines
 * optimization
 */
class TrajectoryGeneration
{
    public:

        /**
         * Typedef for user functions defining
         * the optimization goal
         */
        typedef std::function<Trajectories(
            const Eigen::VectorXd& params)> 
            GenerationFunc;
        typedef std::function<double(
            const Eigen::VectorXd& params)>
            CheckParamsFunc;
        typedef std::function<double(
            const Eigen::Vector3d& trunkPos,
            const Eigen::Vector3d& trunkAxis,
            const Eigen::Vector3d& footPos,
            const Eigen::Vector3d& footAxis)>
            CheckStateFunc;
        typedef std::function<double(
            const HumanoidFixedModel& model)>
            CheckDOFFunc;
        typedef std::function<double(
            double t,
            HumanoidFixedModel& model,
            const Eigen::VectorXd& torques,
            const Eigen::VectorXd& dq,
            const Eigen::VectorXd& ddq,
            bool isDoubleSupport,
            HumanoidFixedModel::SupportFoot supportFoot,
            std::vector<double>& data)>
            ScoreFunc;
        typedef std::function<double(
            const Eigen::VectorXd& params,
            const Trajectories& traj,
            std::vector<double>& data)> 
            EndScoreFunc;

        /**
         * Initialization with 
         * humanoid type
         */
        TrajectoryGeneration(RobotType type);
        
        /**
         * Set the initial parameters for 
         * optimization starting point.
         * Defines the parameter dimension.
         */
        void setInitialParameters(
            const Eigen::VectorXd& params);

        /**
         * Set Trajectory Generation function. 
         * The function returns the Trajectories spline
         * container from given parameters vector.
         */
        void setTrajectoryGenerationFunc(GenerationFunc func);

        /**
         * Set the Parameters check function.
         * The function returns 0.0 if all given 
         * parameters have valid range. It returns
         * positive cost value if given parameters are
         * not in range.
         */
        void setCheckParametersFunc(CheckParamsFunc func);
        
        /**
         * Set the Cartesian Trunk/Foot state 
         * check function.
         * The function returns positive cost value if
         * given trunk/foot position and orientation
         * are outside valid bounds.
         */
        void setCheckStateFunc(CheckStateFunc func);
        
        /**
         * Set the Joint DOF check function.
         * The function returns positive cost value
         * if given model DOF state is outside 
         * valid bounds.
         */
        void setCheckDOFFunc(CheckDOFFunc func);

        /**
         * Set the scoring function.
         * The function returns positive cost value
         * for the given Humanoid model state, DOF
         * torques, velocities and accelerations.
         */
        void setScoreFunc(ScoreFunc func);

        /**
         * set the ending scoring trajectory function.
         * The function is called at the end of Trajectory
         * scoring. It returns positive cost value from given
         * Trajectories spline container;
         */
        void setEndScoreFunc(EndScoreFunc func);

        /**
         * Return initial parameters
         */
        Eigen::VectorXd initialParameters() const;

        /**
         * Call Trajectory Generation function
         */
        Trajectories generateTrajectory(
            const Eigen::VectorXd& params) const;

        /**
         * Call Parameters/State/DOF check function
         */
        double checkParameters(
            const Eigen::VectorXd& params) const;
        double checkState(
            const Eigen::Vector3d& trunkPos,
            const Eigen::Vector3d& trunkAxis,
            const Eigen::Vector3d& footPos,
            const Eigen::Vector3d& footAxis) const;
        double checkDOF(
            const HumanoidFixedModel& model) const;

        /**
         * Call score function
         */
        double score(
            double t,
            HumanoidFixedModel& model,
            const Eigen::VectorXd& torques,
            const Eigen::VectorXd& dq,
            const Eigen::VectorXd& ddq,
            bool isDoubleSupport,
            HumanoidFixedModel::SupportFoot supportFoot,
            std::vector<double>& data) const;
        double endScore(
            const Eigen::VectorXd& params,
            const Trajectories& traj,
            std::vector<double>& data) const;

        /**
         * Build up the Trajectories from 
         * given parameters and evaluates it.
         */
        double scoreTrajectory(
            const Eigen::VectorXd& params) const;
        double scoreTrajectory(
            const Eigen::VectorXd& params,
            const Trajectories& traj) const;

        /**
         * Run the CMA-ES Trajectories optimization
         * with given algorithm configuration.
         */
        void runOptimization(
            unsigned int maxIterations,
            unsigned int restart,
            const std::string& filename = "",
            unsigned int populationSize = 10,
            double lambda = -1.0);

        /**
         * Access to best found Trajectories, 
         * score and parameters
         */
        const Trajectories& bestTrajectories() const;
        const Eigen::VectorXd& bestParameters() const;
        double bestScore() const;

    private:

        /**
         * Humanoid robot type
         */
        RobotType _type;

        /**
         * Initial parameters for 
         * optimization process
         */
        Eigen::VectorXd _initialParameters;

        /**
         * User functions
         */
        GenerationFunc _generateFunc;
        CheckParamsFunc _checkParamsFunc;
        CheckStateFunc _checkStateFunc;
        CheckDOFFunc _checkDOFFunc;
        ScoreFunc _scoreFunc;
        EndScoreFunc _endScoreFunc;

        /**
         * Best found trajectories and 
         * associated score and parameters
         */
        Trajectories _bestTraj;
        Eigen::VectorXd _bestParams;
        double _bestScore;

        /**
         * Counter for CMAES progress function
         */
        long _countIteration;
};

}

#endif

