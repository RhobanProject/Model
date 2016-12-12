#ifndef LEPH_TRAJKICK_HPP
#define LEPH_TRAJKICK_HPP

#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryParameters.hpp"

namespace Leph {

/**
 * TrajKick
 *
 * Template for kick
 * trajectory generation
 */
class TrajKick
{
    public:

        static Eigen::VectorXd initialParameters(
            TrajectoryParameters& trajParams);

        static TrajectoryGeneration::GenerationFunc funcGeneration(
            const TrajectoryParameters& trajParams);

        static TrajectoryGeneration::CheckParamsFunc funcCheckParams(
            const TrajectoryParameters& trajParams);

        static TrajectoryGeneration::CheckStateFunc funcCheckState(
            const TrajectoryParameters& trajParams);

        static TrajectoryGeneration::CheckDOFFunc funcCheckDOF(
            const TrajectoryParameters& trajParams);

        static TrajectoryGeneration::ScoreFunc funcScore(
            const TrajectoryParameters& trajParams);

        static TrajectoryGeneration::EndScoreFunc funcEndScore(
            const TrajectoryParameters& trajParams);
};

}

#endif

