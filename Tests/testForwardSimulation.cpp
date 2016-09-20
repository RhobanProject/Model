#include <iostream>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"

int main()
{
    /*
    Leph::HumanoidModel model(
        Leph::SigmabanModel, 
        "left_foot_tip", false);
    */
    Leph::Model model("../Data/pendulum_triple.urdf");
    model.setDOF("roll1", M_PI/2.0);
    model.setDOF("roll2", -0.8);

    Leph::ForwardSimulation sim(model);

    Leph::ModelViewer viewer(1200, 900);
    Leph::Plot plot;
    double t = 0.0;
    size_t count = 0;
    while (viewer.update()) {
        //sim.goals()(model.getDOFIndex("left_shoulder_pitch")) = 0.8*sin(10*t);
        for (int k=0;k<10;k++) {
            t += 0.0001;
            //sim.goals()(0) = M_PI/2.0 + 0.4*sin(0.2*2.0*3.14*t);
            std::cout << "********** count=" << count << std::endl;
            sim.update(0.0001);
            count++;
            Leph::VectorLabel vect;
            for (size_t i=0;i<model.sizeDOF();i++) {
                vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
                vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
                vect.append("output:" + model.getDOFName(i), sim.outputTorques()(i));
                vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
                vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
                vect.append("active:" + model.getDOFName(i), sim.actives()(i));
                vect.append("input:" + model.getDOFName(i), sim.inputTorques()(i));
                vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
                vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
            }
            plot.add(vect);
        }
        model.setDOFVect(sim.positions());
        Leph::ModelDraw(model, viewer);
    }
    plot.plot("index", "acc:*").render();
    plot.plot("index", "active:*").render();
    plot.plot("index", "output:*").render();
    plot
        .plot("index", "output:*")
        .plot("index", "friction:*")
        .plot("index", "control:*")
        .render();
    plot.plot("index", "input:*").render();
    plot.plot("index", "pos:*").render();
    plot.plot("index", "vel:*").render();
    plot.plot("index", "goal:*").render();

    return 0;
}

