#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/SigmabanModel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

int main()
{
    Leph::SigmabanModel model;

    Leph::ModelViewer viewer(1200, 900);
    viewer.frameLength = 0.02;
    
    Leph::CartWalkProxy walk;
    Leph::VectorLabel params = walk.buildParams();
    params("dynamic:enabled") = 1;
    params("dynamic:step") = 15.0;
    params("dynamic:turn") = 30;

    while (viewer.update()) {
        walk.exec(0.004, params);
        
        Leph::VectorLabel output = walk.lastOutputs()
            .rename("output", "");
        output("left hip pitch") *= -1;
        output("left knee") *= -1;
        output("left foot pitch") *= -1;
        output("left foot roll") *= -1;
        
        model.setDOF(output);
        model.putOnGround();
        
        viewer.addTrackedPoint(model.centerOfMass("origin"));    
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

