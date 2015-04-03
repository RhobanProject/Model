#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/SigmabanFixedModel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

int main()
{
    Leph::SigmabanFixedModel model;

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
        output("left hip roll") *= -1;
        output("left foot pitch") *= -1;
        output("right hip pitch") *= -1;
        output("right knee") *= -1;
        output("right hip roll") *= -1;
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

    return 0;
}

