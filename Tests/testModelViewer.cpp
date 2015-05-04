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
        //Display model
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

