#ifndef LEPH_TRAJKICKSINGLE_HPP
#define LEPH_TRAJKICKSINGLE_HPP

#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryGeneration/TrajectoryParameters.hpp"

namespace Leph {

/**
 * TrajKickSingle
 *
 * Template for single support 
 * kick trajectory generation
 */
class TrajKickSingle
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

