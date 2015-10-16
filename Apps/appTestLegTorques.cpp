#include <iostream>
#include "Model/Model.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/InverseKinematics.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Spline/FittedSpline.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::Plot plot;

    //Load raw model
    Leph::Model model("../Data/leg.urdf");

    //Trajectory splines
    Leph::SmoothSpline splineX;
    Leph::SmoothSpline splineY;
    Leph::SmoothSpline splineZ;
    splineX.addPoint(0.0, -0.02);
    splineX.addPoint(0.5, 0.02);
    splineX.addPoint(1.0, -0.02);
    splineY.addPoint(-0.25, 0.02);
    splineY.addPoint(0.25, -0.02);
    splineY.addPoint(0.75, 0.02);
    splineY.addPoint(1.25, -0.02);
    splineZ.addPoint(0.0, -0.20);
    splineZ.addPoint(0.5, -0.15);
    splineZ.addPoint(1.0, -0.20);

    //Spline container initialization
    std::vector<Leph::FittedSpline> torqueSplines(6);
    
    //Inverse Kinematics
    Leph::InverseKinematics inv(model);
    //Declare model degrees of freedom
    inv.addDOF("left_hip_yaw");
    inv.addDOF("left_hip_pitch");
    inv.addDOF("left_hip_roll");
    inv.addDOF("left_knee");
    inv.addDOF("left_ankle_pitch");
    inv.addDOF("left_ankle_roll");
    //Declare degree of freefom box bounds 
    inv.setLowerBound("left_knee", 0.0);
    
    //Declare target position
    inv.addTargetPosition("foot", "left_foot_tip");
    inv.targetPosition("foot").z() = -0.20;
    //Target orientation
    inv.addTargetOrientation("foot", "left_foot_tip");
        
    Leph::ModelViewer viewer(1200, 900);
    for (double t=0.0;t<2.0;t+=0.01) {
        //Run viewer
        viewer.update();
        //Compute Inverse Kinematics
        inv.run(0.0001, 100);
        int count = 0;
        while (count < 5 && inv.errorSum() > 0.001) {
            std::cout << count << " -> " << inv.errorSum() << std::endl;
            inv.randomDOFNoise(0.01);
            inv.run(0.0001, 100);
            count++;
        }
        //Update target
        inv.targetPosition("foot").x() = splineX.posMod(t);
        inv.targetPosition("foot").y() = splineY.posMod(t);
        inv.targetPosition("foot").z() = splineZ.posMod(t);
        //Display model
        Eigen::Vector3d pt = model.position("left_foot_tip", "origin");
        viewer.addTrackedPoint(pt);    
        Leph::ModelDraw(model, viewer);
        //Foot velocity and acceleration vector
        Eigen::VectorXd vel(6, 1);
        Eigen::VectorXd acc(6, 1);
        vel(0) = 0.0;
        vel(1) = 0.0;
        vel(2) = 0.0;
        vel(3) = splineX.velMod(t);
        vel(4) = splineY.velMod(t);
        vel(5) = splineZ.velMod(t);
        acc(0) = 0.0;
        acc(1) = 0.0;
        acc(2) = 0.0;
        acc(3) = splineX.accMod(t);
        acc(4) = splineY.accMod(t);
        acc(5) = splineZ.accMod(t);
        //Compute foot jacobian matrix
        Eigen::MatrixXd jac = model.pointJacobian("left_foot_tip");
        //Compute joint velocities
        Eigen::VectorXd dq = jac.fullPivLu().solve(vel);
        //Compute joint acceleration
        //acc = J(q)*ddq + dJ(q, dq)*dq
        //=> ddq = J(q)^-1*(acc - dJ*dq)
        //dJ*dq can be computed using pointAcceleration and setting 
        //ddq to zero (thanks Martin Felis !).
        Eigen::VectorXd J_dot_q_dot = model.pointAcceleration("left_foot_tip", dq, 
            Eigen::VectorXd::Zero(model.sizeDOF()));
        Eigen::VectorXd ddq = jac.fullPivLu().solve(acc - J_dot_q_dot);
        //Compute joint torques
        model.setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));
        Eigen::VectorXd torques1 = model.inverseDynamics(dq, ddq);
        model.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
        Eigen::VectorXd torques2 = model.inverseDynamics(dq, ddq);
        Eigen::VectorXd torques = torques1 - torques2;
        //Assign to splines
        torqueSplines[0].addPoint(t, torques(0));
        torqueSplines[1].addPoint(t, torques(1));
        torqueSplines[2].addPoint(t, torques(2));
        torqueSplines[3].addPoint(t, torques(3));
        torqueSplines[4].addPoint(t, torques(4));
        torqueSplines[5].addPoint(t, torques(5));
        //Plot
        plot.add(Leph::VectorLabel(
            "t", t,
            /*
            "q0", model.getDOF("left_hip_yaw"),
            "q1", model.getDOF("left_hip_roll"),
            "q2", model.getDOF("left_hip_pitch"),
            "q3", model.getDOF("left_knee"),
            "q4", model.getDOF("left_ankle_pitch"),
            "q5", model.getDOF("left_ankle_roll")
            */
            "q0", torques(0),
            "q1", torques(1),
            "q2", torques(2),
            "q3", torques(3),
            "q4", torques(4),
            "q5", torques(5)
        ));
    }
    plot.plot("t", "all").render();

    //Splines fitting
    for (size_t i=0;i<torqueSplines.size();i++) {
        //torqueSplines[i].fittingPieces(0.002, false);
        torqueSplines[i].fittingGlobal(4, 10);
    }

    //plot.clear();
    for (double t=0;t<2.0;t+=0.01) {
        plot.add(Leph::VectorLabel(
            "t", t, 
            "q0_", torqueSplines[0].pos(t),
            "q1_", torqueSplines[1].pos(t),
            "q2_", torqueSplines[2].pos(t),
            "q3_", torqueSplines[3].pos(t),
            "q4_", torqueSplines[4].pos(t),
            "q5_", torqueSplines[5].pos(t)
        ));
    }
    plot.plot("t", "all").render();

    return 0;
}

