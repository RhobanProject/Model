#include <iostream>
#include "Model/SigmabanFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Model/InverseKinematics.hpp"
#include "Plot/Plot.hpp"

int main()
{
    //Model with left support foot
    Leph::SigmabanFixedModel model;
    model.setSupportFoot(
        Leph::SigmabanFixedModel::LeftSupportFoot);
    
    //Inverse Kinematics
    Leph::InverseKinematics inv(model.get());
    //Declare model degrees of freedom
    inv.addDOF("right hip pitch");
    inv.addDOF("right hip roll");
    inv.addDOF("right knee");
    inv.addDOF("right foot pitch");
    inv.addDOF("right foot roll");
    inv.addDOF("left hip pitch");
    inv.addDOF("left hip roll");
    inv.addDOF("left knee");
    inv.addDOF("left foot pitch");
    inv.addDOF("left foot roll");
    //Declare target position
    inv.addTargetPosition("flying foot", "right foot tip");
    //target of center of mass
    inv.addTargetCOM();
    inv.targetCOM().z() -= 0.07;
    inv.targetCOM().x() = 0.0;
    //Target orientation
    inv.addTargetOrientation("flying foot", "right foot tip");
    inv.addTargetOrientation("trunk", "trunk");

    //Initial center of mass position
    double yCOM = model.get().centerOfMass("origin").y();
    
    Leph::ModelViewer viewer(1200, 900);
    double freq = 50.0;
    Leph::Scheduling scheduling;
    scheduling.setFrequency(freq);
    double t = 0.0;
    Leph::Plot plot;
    while (viewer.update()) {
        //Display model
        Leph::ModelDraw(model.get(), viewer);
        //Waiting
        scheduling.wait();
    
        //Move target center of mass
        t += 0.02;
        inv.targetCOM().y() = 0.08*sin(t) + yCOM;
        //Update inverse kinematics
        inv.run(0.0001, 100);

        //Compute torques with simple and closed loop model
        Eigen::VectorXd tau1 = model.get().inverseDynamicsClosedLoop(model.get().getFrameIndex("right foot tip"));
        Eigen::VectorXd tau2 = model.get().inverseDynamics();
        plot.add(Leph::VectorLabel(
            "CLOSEDLOOP left foot roll", tau1(model.get().getDOFIndex("left foot roll")),
            "CLOSEDLOOP right foot roll", tau1(model.get().getDOFIndex("right foot roll")),
            "CLOSEDLOOP left hip roll", tau1(model.get().getDOFIndex("left hip roll")),
            "CLOSEDLOOP right hip roll", tau1(model.get().getDOFIndex("right hip roll")),
            "SIMPLE left foot roll", tau2(model.get().getDOFIndex("left foot roll")),
            "SIMPLE left hip roll", tau2(model.get().getDOFIndex("left hip roll")),
            "SIMPLE right hip roll", tau2(model.get().getDOFIndex("right hip roll")),
            "YCOM", 10.0*yCOM,
            "COMY", 10.0*inv.targetCOM().y()
        ));
    }
    plot.plot("index", "all").render();

    return 0;
}

