#include <iostream>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

int main()
{
    Leph::HumanoidModel model(Leph::SigmabanModel, "left_foot_tip", false);

    Leph::ForwardSimulation sim(model);
    sim.position() = model.getDOFVect();

    Leph::ModelViewer viewer(1200, 900);
    while (viewer.update()) {
        sim.update(0.001);
        model.setDOFVect(sim.position());
        //Display model
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

