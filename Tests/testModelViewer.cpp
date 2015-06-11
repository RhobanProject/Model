#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

int main()
{
    Leph::HumanoidFloatingModel model(Leph::SigmabanModel);
    model.putOnGround();

    Leph::ModelViewer viewer(1200, 900);
    viewer.frameLength = 0.02;
    
    Leph::CartWalkProxy walk;
    Leph::VectorLabel params = walk.buildParams();
    params("dynamic:enabled") = 1;
    params("dynamic:step") = 15.0;
    params("dynamic:turn") = 30.0;
        
    double t = 0.0;
    while (viewer.update()) {
        //CartWalk generator
        walk.exec(0.01, params);
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
        model.setDOF(output);
        //Contraint the model on the ground
        model.putOnGround();
        //Display center of mass trajectory
        Eigen::Vector3d com = model.centerOfMass("origin");
        viewer.addTrackedPoint(com);    
        //Display looked at point on the ground
        model.setDOF("tilt", 0.3*sin(2.0*t) + 0.8);
        Eigen::Vector3d lookAtPos1;
        Eigen::Vector3d lookAtPos2;
        Eigen::Vector3d lookAtPos3;
        Eigen::Vector3d lookAtPos4;
        Eigen::Vector3d lookAtPos5;
        Leph::CameraParameters camParams = {80.0*3.14/180.0, 50.0*3.14/180.0};
        model.cameraPixelToWorld(camParams, Eigen::Vector2d(-1.0, -1.0), lookAtPos1);
        model.cameraPixelToWorld(camParams, Eigen::Vector2d(1.0, -1.0), lookAtPos2);
        model.cameraPixelToWorld(camParams, Eigen::Vector2d(1.0, 1.0), lookAtPos3);
        model.cameraPixelToWorld(camParams, Eigen::Vector2d(-1.0, 1.0), lookAtPos4);
        model.cameraPixelToWorld(camParams, Eigen::Vector2d(0.0, 0.0), lookAtPos5);
        viewer.drawLink(model.position("camera", "origin"), lookAtPos1);
        viewer.drawLink(model.position("camera", "origin"), lookAtPos2);
        viewer.drawLink(model.position("camera", "origin"), lookAtPos3);
        viewer.drawLink(model.position("camera", "origin"), lookAtPos4);
        viewer.drawLink(model.position("camera", "origin"), lookAtPos5);
        viewer.drawLink(lookAtPos1, lookAtPos2);
        viewer.drawLink(lookAtPos2, lookAtPos3);
        viewer.drawLink(lookAtPos3, lookAtPos4);
        viewer.drawLink(lookAtPos4, lookAtPos1);
        std::cout << "Horizon at screen height: " << 
            model.cameraScreenHorizon(camParams, 0.0) << std::endl;
        //Display model
        Leph::ModelDraw(model, viewer);
        t += 0.01;
    }

    return 0;
}

