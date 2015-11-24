#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

int main()
{
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);

    Leph::ModelViewer viewer(1200, 900);
    viewer.frameLength = 0.02;
    
    Leph::CartWalkProxy walk;
    Leph::VectorLabel params = walk.buildParams();
    params("dynamic:enabled") = 1;
    params("dynamic:step") = 15.0;
    params("dynamic:turn") = 30.0;
    
    double t = 0.0;
    while (viewer.update()) {
        t += 0.01;
        //CartWalk generator
        walk.exec(0.01, params);
        //Adapt CartWalk convention to Model convention
        Leph::VectorLabel output = walk.lastOutputs()
            .rename("output", "");
        output("left_hip_roll") *= -1;
        output("left_ankle_pitch") *= -1;
        output("right_hip_pitch") *= -1;
        output("right_knee") *= -1;
        output("right_hip_roll") *= -1;
        //Convertion to radian
        output.mulOp(M_PI/180.0);
        //Send motor output to model
        model.get().setDOF(output);
        //Update model floating base
        model.updateBase();
        //Set pitch roll reference for trunk orientation
        model.setOrientation(0.8*sin(t), 0.1);
        //Display center of mass trajectory
        Eigen::Vector3d com = model.get().centerOfMass("origin");
        viewer.addTrackedPoint(com);    
        //Display model
        Leph::ModelDraw(model.get(), viewer);
    }
    
    //Test trunkModelIK
    t = 0.0;
    while (viewer.update()) {
        bool isSuccess = model.trunkModelIK(
            Leph::HumanoidFixedModel::LeftSupportFoot,
            Eigen::Vector3d(0.0, 0.0, 0.2 + 0.05*sin(t)),
            Eigen::Vector3d(0.6, 0.4*sin(t), 0.0),
            Eigen::Vector3d(-0.02, -0.1, 0.05));
        if (!isSuccess) {
            std::cout << "IK ERROR" << std::endl;
        }
        //Display model
        Leph::ModelDraw(model.get(), viewer);
        t += 0.01;
    }

    return 0;
}

