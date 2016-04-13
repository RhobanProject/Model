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
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_hip_yaw"));
        manager.dev<RhAL::DXL>("left_hip_pitch")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_hip_pitch"));
        manager.dev<RhAL::DXL>("left_hip_roll")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_hip_roll"));
        manager.dev<RhAL::DXL>("left_knee")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_knee"));
        manager.dev<RhAL::DXL>("left_ankle_pitch")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_ankle_pitch"));
        manager.dev<RhAL::DXL>("left_ankle_roll")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_ankle_roll"));
        manager.dev<RhAL::DXL>("right_hip_yaw")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_hip_yaw"));
        manager.dev<RhAL::DXL>("right_hip_pitch")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_hip_pitch"));
        manager.dev<RhAL::DXL>("right_hip_roll")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_hip_roll"));
        manager.dev<RhAL::DXL>("right_knee")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_knee"));
        manager.dev<RhAL::DXL>("right_ankle_pitch")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_ankle_pitch"));
        manager.dev<RhAL::DXL>("right_ankle_roll")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_ankle_roll"));
    }
    if (writeArms) {
        manager.dev<RhAL::DXL>("left_shoulder_pitch")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_shoulder_pitch"));
        manager.dev<RhAL::DXL>("left_shoulder_roll")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_shoulder_roll"));
        manager.dev<RhAL::DXL>("left_elbow")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("left_elbow"));
        manager.dev<RhAL::DXL>("right_shoulder_pitch")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_shoulder_pitch"));
        manager.dev<RhAL::DXL>("right_shoulder_roll")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_shoulder_roll"));
        manager.dev<RhAL::DXL>("right_elbow")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("right_elbow"));
    }
    if (writeHead) {
        manager.dev<RhAL::DXL>("head_pitch")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("head_pitch"));
        manager.dev<RhAL::DXL>("head_yaw")
            .goalPosition() = RhAL::Rad2Deg(model.getDOF("head_yaw"));
    }
}

void RhALAppendLog(
    MapSeries& map,
    RhAL::StandardManager& manager)
{
    //DOF names
    std::vector<std::string> names = {
        "head_pitch", "head_yaw",
        "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
        "left_hip_yaw", "left_hip_pitch", "left_hip_roll",
        "left_knee", "left_ankle_pitch", "left_ankle_roll",
        "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
        "right_hip_yaw", "right_hip_pitch", "right_hip_roll",
        "right_knee", "right_ankle_pitch", "right_ankle_roll",
    };
    //Current time
    double time = RhAL::duration_float(RhAL::getTimePoint());
    //Log goal position
    for (const std::string& name : names) {
        map.append(
            "goal:" + name, 
            time, 
            RhAL::Deg2Rad(
                manager.dev<RhAL::DXL>(name)
                .goalPosition().getWrittenValue()));
    }
    //Log position
    for (const std::string& name : names) {
        RhAL::ReadValueFloat value = manager.dev<RhAL::DXL>(name)
            .position().readValue();
        if (!value.isError) {
            map.append(
                "pos:" + name, 
                RhAL::duration_float(value.timestamp), 
                RhAL::Deg2Rad(value.value));
        }
    }
}

}

