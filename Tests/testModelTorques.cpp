#include <iostream>
#include "Model/HumanoidFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Model/InverseKinematics.hpp"
#include "Plot/Plot.hpp"

int main()
{
    //Model with left support foot
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    model.setSupportFoot(
        Leph::HumanoidFixedModel::LeftSupportFoot);
    
    //Inverse Kinematics
    Leph::InverseKinematics inv(model.get());
    //Declare model degrees of freedom
    inv.addDOF("right_hip_pitch");
    inv.addDOF("right_hip_roll");
    inv.addDOF("right_knee");
    inv.addDOF("right_ankle_pitch");
    inv.addDOF("right_ankle_roll");
    inv.addDOF("left_hip_pitch");
    inv.addDOF("left_hip_roll");
    inv.addDOF("left_knee");
    inv.addDOF("left_ankle_pitch");
    inv.addDOF("left_ankle_roll");
    //Declare target position
    inv.addTargetPosition("flying_foot", "right_foot_tip");
    //target of center of mass
    inv.addTargetCOM();
    inv.targetCOM().z() -= 0.04;
    inv.targetCOM().x() = 0.0;
    //Target orientation
    inv.addTargetOrientation("flying_foot", "right_foot_tip");
    inv.addTargetOrientation("trunk", "trunk");

    //Initial center of mass position
    double yCOM = model.get().centerOfMass("origin").y();
    
    Leph::ModelViewer viewer(1200, 900);
    double freq = 50.0;
    Leph::Scheduling scheduling;
    scheduling.setFrequency(freq);
    Leph::Plot plot;
    double t = 0.0;
    while (viewer.update()) {
        //Display model
        Leph::ModelDraw(model.get(), viewer);
        //Waiting
        scheduling.wait();
    
        //Move target center of mass
        inv.targetCOM().y() = 0.08*sin(2.0*3.14*t) + yCOM;
        //Update inverse kinematics
        inv.run(0.0001, 100);

        //Compute torques with simple and closed loop model
        Eigen::VectorXd tau;
        if (t >= 0.0 && t < 3.0) {
            tau = model.get().inverseDynamics();
        }
        else if (t >= 3.0 && t < 6.0) {
            tau = model.get().inverseDynamicsClosedLoop("right_foot_tip", false);
        }
        else if (t >= 6.0 && t < 9.0) {
            tau = model.get().inverseDynamicsClosedLoop("right_foot_tip", true);
        } else {
            break;
        }
        tau(model.get().getDOFIndex("base_x")) = 0.0;
        tau(model.get().getDOFIndex("base_y")) = 0.0;
        tau(model.get().getDOFIndex("base_z")) = 0.0;
        tau(model.get().getDOFIndex("base_yaw")) = 0.0;
        tau(model.get().getDOFIndex("base_pitch")) = 0.0;
        tau(model.get().getDOFIndex("base_roll")) = 0.0;

        //Plot all leg degrees of freedom
        Leph::VectorLabel log;
        for (size_t i=0;i<(size_t)tau.size();i++) {
            if (
                model.get().getDOFName(i).find("elbow") == std::string::npos && 
                model.get().getDOFName(i).find("shoulder") == std::string::npos && 
                model.get().getDOFName(i).find("base") == std::string::npos && 
                model.get().getDOFName(i).find("head") == std::string::npos
            ) {
                log.append(model.get().getDOFName(i), fabs(tau(i)));
            }
        }
        log.append("YCOM", 10.0*yCOM);
        log.append("COMY", 10.0*inv.targetCOM().y());
        log.append("SUM norm 2", tau.norm());
        log.append("SUM norm max", tau.lpNorm<Eigen::Infinity>() + 0.1);
        plot.add(log);
        t += 0.02;
    }
    plot.plot("index", "all").render();

    return 0;
}

