#include <iostream>
#include "Model/HumanoidSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Model/HumanoidFloatingModel.hpp"

/**
 * DOF names
 */
static std::vector<std::string> dofsNames = {
    "head_pitch", "head_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
};

int main()
{
    Leph::HumanoidSimulation sim(Leph::SigmabanModel);
    Leph::ModelViewer viewer(1200, 900);
    Leph::HumanoidFloatingModel goalModel(Leph::SigmabanModel);
    
    //Initialing IKWalk
    Leph::IKWalk::Parameters params;
    params.freq = 1.5;
    params.enabledGain = 1.0;
    params.supportPhaseRatio = 0.6;
    params.footYOffset = 0.02;
    params.stepGain = 0.05;
    params.riseGain = 0.06;
    params.turnGain = 0.0;
    params.lateralGain = 0.0;
    params.trunkZOffset = 0.05;
    params.swingGain = 0.02;
    params.swingRollGain = 0.0;
    params.swingPhase = 0.0;
    params.stepUpVel = 2.0;
    params.stepDownVel = 2.0;
    params.riseUpVel = 2.0;
    params.riseDownVel = 2.0;
    params.swingPause = 0.1;
    params.swingVel = 5.0;
    params.trunkXOffset = 0.0;
    params.trunkYOffset = 0.0;
    params.trunkPitch = 0.25;
    params.trunkRoll = 0.0;
    params.extraLeftX = 0.0;
    params.extraLeftY = 0.0;
    params.extraLeftZ = 0.0;
    params.extraLeftYaw = 0.0;
    params.extraLeftPitch = 0.0;
    params.extraLeftRoll = 0.0;
    params.extraRightX = 0.0;
    params.extraRightY = 0.0;
    params.extraRightZ = 0.0;
    params.extraRightYaw = 0.0;
    params.extraRightPitch = 0.0;
    params.extraRightRoll = 0.0;

    double phase = 0.0;
    bool isFirstIteration = true;
    while (viewer.update()) {
        //Run walk generator
        Leph::IKWalk::walk(goalModel, params, phase, 0.01);
        //Import target position
        for (const std::string& name : dofsNames) {
            sim.setGoal(name, goalModel.getDOF(name));
        }
        if (isFirstIteration) {
            for (const std::string& name : dofsNames) {
                sim.setPos(name, goalModel.getDOF(name));
            }
            sim.putOnGround();
            isFirstIteration = false;
        }

        for (int k=0;k<100;k++) {
            sim.update(0.0001);
        }
        Leph::ModelDraw(sim.model(), viewer);
        Leph::CleatsDraw(sim, viewer);
        std::cout << sim.getWeightSum() << " " << sim.getWeightLeftRatio() << std::endl;
    }

    return 0;
}

