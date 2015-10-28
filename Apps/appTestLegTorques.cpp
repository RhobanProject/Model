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
    //Parameters
    double period = 3.0;
    double sizeMvt = 0.04;
    double polynomLength = 0.5;
    double xOffset = 0.10;

    //Plot instances
    Leph::Plot plotPosition;
    Leph::Plot plotTorques;

    //Load raw model
    Leph::Model model("../Data/leg.urdf");

    //Trajectory splines
    Leph::SmoothSpline splineX;
    Leph::SmoothSpline splineY;
    Leph::SmoothSpline splineZ;
    splineX.addPoint(0.0, -sizeMvt + xOffset);
    splineX.addPoint(0.5, sizeMvt + xOffset);
    splineX.addPoint(1.0, -sizeMvt + xOffset);
    splineY.addPoint(-0.25, sizeMvt);
    splineY.addPoint(0.25, -sizeMvt);
    splineY.addPoint(0.75, sizeMvt);
    splineY.addPoint(1.25, -sizeMvt);
    splineZ.addPoint(0.0, -0.20);
    splineZ.addPoint(0.5, -0.15);
    splineZ.addPoint(1.0, -0.20);

    //Spline container initialization
    std::vector<Leph::FittedSpline> torqueSplines(6);
    std::vector<Leph::FittedSpline> positionSplines(6);
    
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
    double t = 0.0;
    while (viewer.update()) {
        //Update target
        inv.targetPosition("foot").x() = splineX.posMod(t/period);
        inv.targetPosition("foot").y() = splineY.posMod(t/period);
        inv.targetPosition("foot").z() = splineZ.posMod(t/period);
        //Compute Inverse Kinematics
        inv.run(0.0001, 100);
        int count = 0;
        while (count < 5 && inv.errorSum() > 0.001) {
            std::cout << count << " -> " << inv.errorSum() << std::endl;
            inv.randomDOFNoise(0.01);
            inv.run(0.0001, 100);
            count++;
        }
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
        vel(3) = splineX.velMod(t/period);
        vel(4) = splineY.velMod(t/period);
        vel(5) = splineZ.velMod(t/period);
        acc(0) = 0.0;
        acc(1) = 0.0;
        acc(2) = 0.0;
        acc(3) = splineX.accMod(t/period);
        acc(4) = splineY.accMod(t/period);
        acc(5) = splineZ.accMod(t/period);
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
        Eigen::VectorXd pos = model.getDOFVect();
        //Time update
        t += 0.01;
        if (t > period) continue;
        //Assign to splines
        torqueSplines[0].addPoint(t, torques(0));
        torqueSplines[1].addPoint(t, torques(1));
        torqueSplines[2].addPoint(t, torques(2));
        torqueSplines[3].addPoint(t, torques(3));
        torqueSplines[4].addPoint(t, torques(4));
        torqueSplines[5].addPoint(t, torques(5));
        positionSplines[0].addPoint(t, pos(0));
        positionSplines[1].addPoint(t, pos(1));
        positionSplines[2].addPoint(t, pos(2));
        positionSplines[3].addPoint(t, pos(3));
        positionSplines[4].addPoint(t, pos(4));
        positionSplines[5].addPoint(t, pos(5));
        //Plot
        plotTorques.add(Leph::VectorLabel(
            "t", t,
            "t0", torques(0),
            "t1", torques(1),
            "t2", torques(2),
            "t3", torques(3),
            "t4", torques(4),
            "t5", torques(5)
        ));
        plotPosition.add(Leph::VectorLabel(
            "t", t,
            "q0", pos(0),
            "q1", pos(1),
            "q2", pos(2),
            "q3", pos(3),
            "q4", pos(4),
            "q5", pos(5)
        ));
    }

    //Splines fitting
    for (size_t i=0;i<torqueSplines.size();i++) {
        torqueSplines[i].fittingGlobal(4, (int)(polynomLength*100.0));
        positionSplines[i].fittingGlobal(4, (int)(polynomLength*100.0));
    }
    //Dumping splines
    std::cout << "=== Torque Trajectories" << std::endl;
    for (size_t i=0;i<torqueSplines.size();i++) {
        std::cout << "motor_" << i << std::endl;
        for (size_t j=0;j<torqueSplines[i].size();j++) {
            std::cout << torqueSplines[i].part(j).max-torqueSplines[i].part(j).min << std::endl;
            for (size_t k=0;k<torqueSplines[i].part(j).polynom.degree();k++) {
                std::cout << torqueSplines[i].part(j).polynom(k) << ", ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << "=== Position Trajectories" << std::endl;
    for (size_t i=0;i<positionSplines.size();i++) {
        std::cout << "motor_" << i << std::endl;
        for (size_t j=0;j<positionSplines[i].size();j++) {
            std::cout << positionSplines[i].part(j).max-positionSplines[i].part(j).min << std::endl;
            for (size_t k=0;k<positionSplines[i].part(j).polynom.degree();k++) {
                std::cout << positionSplines[i].part(j).polynom(k) << ", ";
            }
            std::cout << std::endl;
        }
    }

    //Ploting position and torque trajectories
    for (double t=0;t<period;t+=0.02) {
        plotTorques.add(Leph::VectorLabel(
            "t", t, 
            "t0 fitted", torqueSplines[0].pos(t),
            "t1 fitted", torqueSplines[1].pos(t),
            "t2 fitted", torqueSplines[2].pos(t),
            "t3 fitted", torqueSplines[3].pos(t),
            "t4 fitted", torqueSplines[4].pos(t),
            "t5 fitted", torqueSplines[5].pos(t)
        ));
        plotPosition.add(Leph::VectorLabel(
            "t", t, 
            "q0 fitted", positionSplines[0].pos(t),
            "q1 fitted", positionSplines[1].pos(t),
            "q2 fitted", positionSplines[2].pos(t),
            "q3 fitted", positionSplines[3].pos(t),
            "q4 fitted", positionSplines[4].pos(t),
            "q5 fitted", positionSplines[5].pos(t)
        ));
    }
    plotPosition.plot("t", "all").render();
    plotTorques.plot("t", "all").render();

    return 0;
}

