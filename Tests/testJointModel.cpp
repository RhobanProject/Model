#include <iostream>
#include <cassert>
#include <cmath>
#include "Model/JointModel.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::Plot plot;
    Leph::JointModel joint(Leph::JointModel::JointActuated, "test");
    assert(joint.getName() == "test");

    Eigen::VectorXd params = joint.getParameters();
    params(7) = 0.1;
    joint.setParameters(params);

    plot.clear();
    std::cout << "Zero friction torque: " << joint.frictionTorque(0.0, 0.0) << std::endl;
    for (double t=0.0;t<2.0;t+=0.005) {
        double goal = sin(t*2.0*3.14*3.0)*sin(t*2.0*3.14*0.2) + sin(t*2.0*3.14*2.0);
        joint.updateState(0.005, goal, 0.0, 0.0);
        plot.add(Leph::VectorLabel(
            "t", t,
            "goal", goal,
            "delayedGoal", joint.getDelayedGoal()
        )); 
    }
    plot.plot("t", "all").render();
    
    plot.clear();
    for (double vel=-3.0;vel<3.0;vel+=0.01) {
        plot.add(Leph::VectorLabel(
            "vel", vel,
            "friction", joint.frictionTorque(0.0, vel)
        )); 
    }
    plot.plot("vel", "friction").render();

    return 0;
}

