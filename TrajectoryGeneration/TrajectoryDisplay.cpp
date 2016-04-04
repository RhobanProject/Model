#include "TrajectoryGeneration/TrajectoryDisplay.h"
#include "Plot/Plot.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Model/MotorModel.hpp"

namespace Leph {

void TrajectoriesDisplay(
    const Trajectories& traj, RobotType type)
{
    //Plot Cartesian trajectory
    Plot plot;
    for (double t=traj.min();t<=traj.max();t+=0.01) {
        plot.add(VectorLabel(
            "time", t,
            "is_double_support", traj.get("is_double_support").pos(t),
            "is_left_support_foot", traj.get("is_left_support_foot").pos(t)
        ));
        if (traj.exist("base_pitch")) {
            plot.add(VectorLabel(
                "time", t,
                "base_pitch", traj.get("base_pitch").pos(t),
                "base_pitch_vel", traj.get("base_pitch").vel(t)
            ));
        }
        if (traj.exist("base_roll")) {
            plot.add(VectorLabel(
                "time", t,
                "base_roll", traj.get("base_roll").pos(t),
                "base_roll_vel", traj.get("base_roll").vel(t)
            ));
        }
    }
    plot.plot("time", "all").render();
    plot.clear();
    for (double t=traj.min();t<=traj.max();t+=0.01) {
        plot.add(VectorLabel(
            "time", t,
            "trunk_pos_x", traj.get("trunk_pos_x").pos(t),
            "trunk_pos_y", traj.get("trunk_pos_y").pos(t),
            "trunk_pos_z", traj.get("trunk_pos_z").pos(t),
            "trunk_pos_x_vel", traj.get("trunk_pos_x").vel(t),
            "trunk_pos_y_vel", traj.get("trunk_pos_y").vel(t),
            "trunk_pos_z_vel", traj.get("trunk_pos_z").vel(t)
        ));
    }
    plot.plot("time", "all").render();
    plot.clear();
    for (double t=traj.min();t<=traj.max();t+=0.01) {
        plot.add(VectorLabel(
            "time", t,
            "trunk_axis_x", traj.get("trunk_axis_x").pos(t),
            "trunk_axis_y", traj.get("trunk_axis_y").pos(t),
            "trunk_axis_z", traj.get("trunk_axis_z").pos(t),
            "trunk_axis_x_vel", traj.get("trunk_axis_x").vel(t),
            "trunk_axis_y_vel", traj.get("trunk_axis_y").vel(t),
            "trunk_axis_z_vel", traj.get("trunk_axis_z").vel(t)
        ));
    }
    plot.plot("time", "all").render();
    plot.clear();
    for (double t=traj.min();t<=traj.max();t+=0.01) {
        plot.add(VectorLabel(
            "time", t,
            "foot_pos_x", traj.get("foot_pos_x").pos(t),
            "foot_pos_y", traj.get("foot_pos_y").pos(t),
            "foot_pos_z", traj.get("foot_pos_z").pos(t),
            "foot_pos_x_vel", traj.get("foot_pos_x").vel(t),
            "foot_pos_y_vel", traj.get("foot_pos_y").vel(t),
            "foot_pos_z_vel", traj.get("foot_pos_z").vel(t)
        ));
    }
    plot.plot("time", "all").render();
    plot.clear();
    //Display Trajectory
    ModelViewer viewer(1200, 900);
    HumanoidFixedModel model(type);
    Scheduling scheduling;
    scheduling.setFrequency(50.0);
    double t = traj.min();
    bool isLoop = false;
    double sumTorques = 0.0;
    double sumZMP = 0.0;
    double maxZMP = -1.0;
    double waiting = 0.0;
    while (viewer.update()) {
        Eigen::VectorXd dq;
        Eigen::VectorXd ddq;
        bool isIKSuccess = TrajectoriesComputeKinematics(
            t, traj, model, dq, ddq);
        if (!isIKSuccess) {
            std::cout << "IK ERROR t=" << t << std::endl;
            t += 0.01;
            continue;
        }
        //Compute DOF torques
        bool isDoubleSupport;
        HumanoidFixedModel::SupportFoot supportFoot;
        TrajectoriesSupportFootState(t, traj,
            isDoubleSupport, supportFoot);
        Eigen::VectorXd torques;
        if (isDoubleSupport) {
            if (supportFoot == HumanoidFixedModel::LeftSupportFoot) {
                torques = model.get().inverseDynamicsClosedLoop(
                    "right_foot_tip", false, dq, ddq);
            } else {
                torques = model.get().inverseDynamicsClosedLoop(
                    "left_foot_tip", false, dq, ddq);
            }
        } else {
            torques = model.get().inverseDynamics(dq, ddq);
        }
        //Compute ZMP
        Eigen::Vector3d zmp;
        if (isDoubleSupport) {
            zmp = model.get().centerOfMass("origin");
        } else {
            zmp = model.zeroMomentPointFromTorques("origin", torques);
        }
        zmp.z() = 0.0;
        //Disable base DOF
        torques(model.get().getDOFIndex("base_x")) = 0.0;
        torques(model.get().getDOFIndex("base_y")) = 0.0;
        torques(model.get().getDOFIndex("base_z")) = 0.0;
        torques(model.get().getDOFIndex("base_yaw")) = 0.0;
        torques(model.get().getDOFIndex("base_pitch")) = 0.0;
        torques(model.get().getDOFIndex("base_roll")) = 0.0;
        //Compute voltage
        Eigen::VectorXd volts = MotorModel::voltage(dq, torques);
        //Display ZMP and trunk/foot trajectory
        if (isDoubleSupport) {
            viewer.addTrackedPoint(
                zmp, ModelViewer::Red);
        } else {
            viewer.addTrackedPoint(
                zmp, ModelViewer::Yellow);
        }
        viewer.addTrackedPoint(
            model.get().position("trunk", "origin"), 
            ModelViewer::Purple);
        viewer.addTrackedPoint(
            model.get().position("right_foot_tip", "origin"), 
            ModelViewer::Cyan);
        //Model display
        ModelDraw(model.get(), viewer);
        //Scheduling
        scheduling.wait();
        if (waiting > 0.0) {
            waiting -= 0.01;
        } else {
            if (t >= traj.max()) {
                t = traj.min();
                isLoop = true;
                waiting = 1.0;
            } else {
                t += 0.01;
            }
        }
        if (!isLoop) {
            plot.add(VectorLabel(
                "t", t,
                "left_torque:hip_yaw", 
                    torques(model.get().getDOFIndex("left_hip_yaw")),
                "left_torque:hip_pitch", 
                    torques(model.get().getDOFIndex("left_hip_pitch")),
                "left_torque:hip_roll", 
                    torques(model.get().getDOFIndex("left_hip_roll")),
                "left_torque:knee", 
                    torques(model.get().getDOFIndex("left_knee")),
                "left_torque:ankle_pitch", 
                    torques(model.get().getDOFIndex("left_ankle_pitch")),
                "left_torque:ankle_roll", 
                    torques(model.get().getDOFIndex("left_ankle_roll")),
                "right_torque:hip_yaw", 
                    torques(model.get().getDOFIndex("right_hip_yaw")),
                "right_torque:hip_pitch", 
                    torques(model.get().getDOFIndex("right_hip_pitch")),
                "right_torque:hip_roll", 
                    torques(model.get().getDOFIndex("right_hip_roll")),
                "right_torque:knee", 
                    torques(model.get().getDOFIndex("right_knee")),
                "right_torque:ankle_pitch", 
                    torques(model.get().getDOFIndex("right_ankle_pitch")),
                "right_torque:ankle_roll", 
                    torques(model.get().getDOFIndex("right_ankle_roll")),
                "left_volt:hip_yaw", 
                    volts(model.get().getDOFIndex("left_hip_yaw")),
                "left_volt:hip_pitch", 
                    volts(model.get().getDOFIndex("left_hip_pitch")),
                "left_volt:hip_roll", 
                    volts(model.get().getDOFIndex("left_hip_roll")),
                "left_volt:knee", 
                    volts(model.get().getDOFIndex("left_knee")),
                "left_volt:ankle_pitch", 
                    volts(model.get().getDOFIndex("left_ankle_pitch")),
                "left_volt:ankle_roll", 
                    volts(model.get().getDOFIndex("left_ankle_roll")),
                "right_volt:hip_yaw", 
                    volts(model.get().getDOFIndex("right_hip_yaw")),
                "right_volt:hip_pitch", 
                    volts(model.get().getDOFIndex("right_hip_pitch")),
                "right_volt:hip_roll", 
                    volts(model.get().getDOFIndex("right_hip_roll")),
                "right_volt:knee", 
                    volts(model.get().getDOFIndex("right_knee")),
                "right_volt:ankle_pitch", 
                    volts(model.get().getDOFIndex("right_ankle_pitch")),
                "right_volt:ankle_roll", 
                    volts(model.get().getDOFIndex("right_ankle_roll")),
                "zmp_x", zmp.x(),
                "zmp_y", zmp.y()
            ));
            sumTorques += 0.01*torques.norm();
            sumZMP += 0.01*zmp.norm();
            if (maxZMP < 0.0 || maxZMP < zmp.lpNorm<Eigen::Infinity>()) {
                maxZMP = zmp.lpNorm<Eigen::Infinity>();
            }
        }
    }
    //Display dynamics info
    std::cout << "Mean Torques Norm: " << sumTorques << std::endl;
    std::cout << "Mean ZMP Norm: " << sumZMP << std::endl;
    std::cout << "Max ZMP: " << maxZMP << std::endl;
    //Plot DOF torques and ZMP
    plot.plot("t", "left_torque:*").render();
    plot.plot("t", "right_torque:*").render();
    plot.plot("t", "left_volt:*").render();
    plot.plot("t", "right_volt:*").render();
    plot.plot("t", "zmp_x").plot("t", "zmp_y").render();
}

}

