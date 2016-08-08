#include <iostream>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

int main()
{
    Leph::HumanoidModel model(
        Leph::SigmabanModel, 
        "left_foot_tip", false);
    //Leph::Model model("../Data/pendulum.urdf");

    Leph::ForwardSimulation sim(model);

    Leph::ModelViewer viewer(1200, 900);
    double t = 0.0;
    while (viewer.update()) {
        t += 0.01;
        sim.goals()(model.getDOFIndex("left_shoulder_pitch")) = 0.8*sin(10*t);
        for (int k=0;k<100;k++) {
            sim.update(0.0001);
        }
        model.setDOFVect(sim.positions());
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

