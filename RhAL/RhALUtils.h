#ifndef LEPH_RHALUTILS_H
#define LEPH_RHALUTILS_H

#include <RhAL.hpp>
#include "Model/HumanoidModel.hpp"

namespace Leph {

/**
 * Write to given RhAL Manager the
 * degrees of freedom target from given
 * Humanoid Model.
 * writeLegs, writeArms, writeHead flags
 * enable writing the DOF of legs, arms
 * or the head.
 */
void RhALWriteStateGoal(
    RhAL::StandardManager& manager,
    const HumanoidModel& model,
    bool writeLegs,
    bool writeArms,
    bool writeHead);

}

#endif

