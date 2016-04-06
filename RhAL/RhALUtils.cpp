#include "RhAL/RhALUtils.h"

namespace Leph {

void RhALWriteStateGoal(
    RhAL::StandardManager& manager,
    const HumanoidModel& model,
    bool writeLegs,
    bool writeArms,
    bool writeHead)
{
    if (writeLegs) {
        manager.dev<RhAL::DXL>("left_hip_yaw")
            .setGoalPositionRad(model.getDOF("left_hip_yaw"));
        manager.dev<RhAL::DXL>("left_hip_pitch")
            .setGoalPositionRad(model.getDOF("left_hip_pitch"));
        manager.dev<RhAL::DXL>("left_hip_roll")
            .setGoalPositionRad(model.getDOF("left_hip_roll"));
        manager.dev<RhAL::DXL>("left_knee")
            .setGoalPositionRad(model.getDOF("left_knee"));
        manager.dev<RhAL::DXL>("left_ankle_pitch")
            .setGoalPositionRad(model.getDOF("left_ankle_pitch"));
        manager.dev<RhAL::DXL>("left_ankle_roll")
            .setGoalPositionRad(model.getDOF("left_ankle_roll"));
        manager.dev<RhAL::DXL>("right_hip_yaw")
            .setGoalPositionRad(model.getDOF("right_hip_yaw"));
        manager.dev<RhAL::DXL>("right_hip_pitch")
            .setGoalPositionRad(model.getDOF("right_hip_pitch"));
        manager.dev<RhAL::DXL>("right_hip_roll")
            .setGoalPositionRad(model.getDOF("right_hip_roll"));
        manager.dev<RhAL::DXL>("right_knee")
            .setGoalPositionRad(model.getDOF("right_knee"));
        manager.dev<RhAL::DXL>("right_ankle_pitch")
            .setGoalPositionRad(model.getDOF("right_ankle_pitch"));
        manager.dev<RhAL::DXL>("right_ankle_roll")
            .setGoalPositionRad(model.getDOF("right_ankle_roll"));
    }
    if (writeArms) {
        manager.dev<RhAL::DXL>("left_shoulder_pitch")
            .setGoalPositionRad(model.getDOF("left_shoulder_pitch"));
        manager.dev<RhAL::DXL>("left_shoulder_roll")
            .setGoalPositionRad(model.getDOF("left_shoulder_roll"));
        manager.dev<RhAL::DXL>("left_elbow")
            .setGoalPositionRad(model.getDOF("left_elbow"));
        manager.dev<RhAL::DXL>("right_shoulder_pitch")
            .setGoalPositionRad(model.getDOF("right_shoulder_pitch"));
        manager.dev<RhAL::DXL>("right_shoulder_roll")
            .setGoalPositionRad(model.getDOF("right_shoulder_roll"));
        manager.dev<RhAL::DXL>("right_elbow")
            .setGoalPositionRad(model.getDOF("right_elbow"));
    }
    if (writeHead) {
        manager.dev<RhAL::DXL>("head_pitch")
            .setGoalPositionRad(model.getDOF("head_pitch"));
        manager.dev<RhAL::DXL>("head_yaw")
            .setGoalPositionRad(model.getDOF("head_yaw"));
    }
}

}

