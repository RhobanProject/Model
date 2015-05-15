#include <iostream>
#include "IKWalk/IKWalk.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"

int main()
{
    //Viewer and model
    Leph::ModelViewer viewer(1200, 900);
    Leph::HumanoidFloatingModel modelOld(Leph::SigmabanModel);
    modelOld.putOnGround();
    modelOld.setStatePosY(0.20);
    Leph::HumanoidFixedModel modelNew(Leph::SigmabanModel);
    modelNew.updateBase();
    
    //Initialing CartWalk generator
    Leph::CartWalkProxy walk;
    Leph::VectorLabel paramsOld = walk.buildParams();
    paramsOld("dynamic:enabled") = 1;
    paramsOld("static:timeGain") = 0.5;
    paramsOld("dynamic:step") = 0.0;
    paramsOld("dynamic:turn") = 10.0;
    paramsOld("dynamic:step") = 4.0;
    paramsOld("static:riseGain") = 3.0;
    paramsOld("static:swingPhase") = 0.0;
    paramsOld("static:swingGain") = 3.0;
    paramsOld("static:zOffset") = 4.0;
    paramsOld("static:yLat") = 0.0;
    paramsOld("static:yOffset") = 0.0;

    //Initialing IKWalk
    Leph::IKWalk::Parameters params;
    params.freq = 0.3;
    params.enabledGain = 1.0;
    params.supportPhaseRatio = 0.6;
    params.footYOffset = 0.01;
    params.stepGain = 0.08;
    params.riseGain = 0.06;
    params.turnGain = -0.3;
    params.lateralGain = 0.0;
    params.trunkZOffset = 0.08;
    params.swingGain = 0.05;
    params.swingPhase = 0.0;
    params.stepUpVel = 2.0;
    params.stepDownVel = 2.0;
    params.riseUpVel = 2.0;
    params.riseDownVel = 2.0;
    params.swingPause = 0.1;
    params.swingVel = 5.0;
    params.trunkXOffset = 0.0;
    params.trunkYOffset = 0.0;
    params.trunkPitch = 0.2;
    params.trunkRoll = -0.4;
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
    
    Leph::Scheduling scheduling(50.0);
    double t = 0.0;
    while (viewer.update()) {
        t += 0.01;
        //CartWalk generator
        walk.exec(0.01, paramsOld);
        //Adapt CartWalk convention to Model convention
        Leph::VectorLabel output = walk.lastOutputs()
            .rename("output", "");
        output("left hip roll") *= -1;
        output("left foot pitch") *= -1;
        output("right hip pitch") *= -1;
        output("right knee") *= -1;
        output("right hip roll") *= -1;
        //Convertion to radian
        output.mulOp(M_PI/180.0);
        //Send motor output to model
        modelOld.setDOF(output);
        //Contraint the model on the ground
        modelOld.putOnGround();
        //Track moving points
        viewer.addTrackedPoint(
            modelOld.position("right foot tip", "origin"), 
            Leph::ModelViewer::Yellow);
        viewer.addTrackedPoint(
            modelOld.position("trunk", "origin"), 
            Leph::ModelViewer::Purple);
        viewer.addTrackedPoint(
            modelOld.centerOfMass("origin"), 
            Leph::ModelViewer::Cyan);
        //Display model
        Leph::ModelDraw(modelOld, viewer);
        
        //Add some trunk orientation oscillation
        params.trunkPitch = 0.2*sin(t*2.0);
        params.trunkRoll = 0.1*sin(t*3.0);
        //Run walk generator
        bool success = Leph::IKWalk::walk(
            modelNew.get(), params, phase, 0.01);
        if (!success) {
            std::cout << "IKWalk inverse kinematics failed" << std::endl;
        }
        //Contraint the model on the ground
        modelNew.updateBase();
        //Track moving points
        viewer.addTrackedPoint(
            modelNew.get().position("left foot tip", "origin"), 
            Leph::ModelViewer::Red);
        viewer.addTrackedPoint(
            modelNew.get().position("trunk", "origin"), 
            Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            modelNew.get().centerOfMass("origin"), 
            Leph::ModelViewer::Blue);
        //Display model
        Leph::ModelDraw(modelNew.get(), viewer);
        //Waiting
        scheduling.wait();
    }

    return 0;
}

