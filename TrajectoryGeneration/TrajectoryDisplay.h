#ifndef LEPH_TRAJECTORYDISPLAY_H
#define LEPH_TRAJECTORYDISPLAY_H

#include "Model/HumanoidFixedModel.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"

namespace Leph {

/**
 * Plot and Display the given Trajectories
 * spline container.
 */
void TrajectoriesDisplay(
    const Trajectories& traj, 
    RobotType type = SigmabanModel);

}

#endif

