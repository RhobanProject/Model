#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

namespace Leph {

Trajectories TrajectoriesInit()
{
    SplineContainer<SmoothSpline> traj;
    traj.add("is_double_support");
    traj.add("is_left_support_foot");
    traj.add("trunk_pos_x");
    traj.add("trunk_pos_y");
    traj.add("trunk_pos_z");
    traj.add("trunk_axis_x");
    traj.add("trunk_axis_y");
    traj.add("trunk_axis_z");
    traj.add("foot_pos_x");
    traj.add("foot_pos_y");
    traj.add("foot_pos_z");
    traj.add("foot_axis_x");
    traj.add("foot_axis_y");
    traj.add("foot_axis_z");

    return traj;
}

void TrajectoriesTrunkFootPos(
    double t, const Trajectories& traj,
    Eigen::Vector3d& trunkPos,
    Eigen::Vector3d& trunkAxis,
    Eigen::Vector3d& footPos,
    Eigen::Vector3d& footAxis)
{
    //Compute Cartesian positions
    trunkPos = Eigen::Vector3d(
        traj.get("trunk_pos_x").pos(t), 
        traj.get("trunk_pos_y").pos(t), 
        traj.get("trunk_pos_z").pos(t));
    trunkAxis = Eigen::Vector3d(
        traj.get("trunk_axis_x").pos(t), 
        traj.get("trunk_axis_y").pos(t), 
        traj.get("trunk_axis_z").pos(t));
    footPos = Eigen::Vector3d(
        traj.get("foot_pos_x").pos(t), 
        traj.get("foot_pos_y").pos(t), 
        traj.get("foot_pos_z").pos(t));
    footAxis = Eigen::Vector3d(
        traj.get("foot_axis_x").pos(t), 
        traj.get("foot_axis_y").pos(t), 
        traj.get("foot_axis_z").pos(t));
}
void TrajectoriesTrunkFootVel(
    double t, const Trajectories& traj,
    Eigen::Vector3d& trunkPosVel,
    Eigen::Vector3d& trunkAxisVel,
    Eigen::Vector3d& footPosVel,
    Eigen::Vector3d& footAxisVel)
{
    //Compute Cartesian velocities
    trunkPosVel = Eigen::Vector3d(
        traj.get("trunk_pos_x").vel(t), 
        traj.get("trunk_pos_y").vel(t), 
        traj.get("trunk_pos_z").vel(t));
    trunkAxisVel = Eigen::Vector3d(
        traj.get("trunk_axis_x").vel(t), 
        traj.get("trunk_axis_y").vel(t), 
        traj.get("trunk_axis_z").vel(t));
    footPosVel = Eigen::Vector3d(
        traj.get("foot_pos_x").vel(t), 
        traj.get("foot_pos_y").vel(t), 
        traj.get("foot_pos_z").vel(t));
    footAxisVel = Eigen::Vector3d(
        traj.get("foot_axis_x").vel(t), 
        traj.get("foot_axis_y").vel(t), 
        traj.get("foot_axis_z").vel(t));
}
void TrajectoriesTrunkFootAcc(
    double t, const Trajectories& traj,
    Eigen::Vector3d& trunkPosAcc,
    Eigen::Vector3d& trunkAxisAcc,
    Eigen::Vector3d& footPosAcc,
    Eigen::Vector3d& footAxisAcc)
{
    //Compute Cartesian accelerations
    trunkPosAcc = Eigen::Vector3d(
        traj.get("trunk_pos_x").acc(t), 
        traj.get("trunk_pos_y").acc(t), 
        traj.get("trunk_pos_z").acc(t));
    trunkAxisAcc = Eigen::Vector3d(
        traj.get("trunk_axis_x").acc(t), 
        traj.get("trunk_axis_y").acc(t), 
        traj.get("trunk_axis_z").acc(t));
    footPosAcc = Eigen::Vector3d(
        traj.get("foot_pos_x").acc(t), 
        traj.get("foot_pos_y").acc(t), 
        traj.get("foot_pos_z").acc(t));
    footAxisAcc = Eigen::Vector3d(
        traj.get("foot_axis_x").acc(t), 
        traj.get("foot_axis_y").acc(t), 
        traj.get("foot_axis_z").acc(t));
}
void TrajectoriesSupportFootState(
    double t, const Trajectories& traj,
    bool& isDoubleSupport, 
    HumanoidFixedModel::SupportFoot& supportFoot)
{
    //Compute support foot state
    isDoubleSupport = (
        traj.get("is_double_support").pos(t) >= 0.5 ?
        true : false);
    supportFoot = (
        traj.get("is_left_support_foot").pos(t) >= 0.5 ?
        HumanoidFixedModel::LeftSupportFoot : 
        HumanoidFixedModel::RightSupportFoot);
}

bool TrajectoriesComputeKinematics(
    double t, const Trajectories& traj,
    Leph::HumanoidFixedModel& model, 
    Eigen::VectorXd& dq, Eigen::VectorXd& ddq)
{
    //Compute Cartesian target
    Eigen::Vector3d trunkPos;
    Eigen::Vector3d trunkAxis;
    Eigen::Vector3d footPos;
    Eigen::Vector3d footAxis;
    Eigen::Vector3d trunkPosVel;
    Eigen::Vector3d trunkAxisVel;
    Eigen::Vector3d footPosVel;
    Eigen::Vector3d footAxisVel;
    Eigen::Vector3d trunkPosAcc;
    Eigen::Vector3d trunkAxisAcc;
    Eigen::Vector3d footPosAcc;
    Eigen::Vector3d footAxisAcc;
    bool isDoubleSupport;
    HumanoidFixedModel::SupportFoot supportFoot;
    TrajectoriesTrunkFootPos(t, traj, 
        trunkPos, trunkAxis, footPos, footAxis);
    TrajectoriesTrunkFootVel(t, traj, 
        trunkPosVel, trunkAxisVel, footPosVel, footAxisVel);
    TrajectoriesTrunkFootAcc(t, traj, 
        trunkPosAcc, trunkAxisAcc, footPosAcc, footAxisAcc);
    TrajectoriesSupportFootState(t, traj,
        isDoubleSupport, supportFoot);
    //Compute DOF positions
    bool isSuccess = model.trunkFootIK(
        supportFoot,
        trunkPos,
        Leph::AxisToMatrix(trunkAxis),
        footPos,
        Leph::AxisToMatrix(footAxis));
    if (!isSuccess) {
        return false;
    }
    //Axis differentiation is converted in proper angular
    //velocity and acceleration
    //Compute DOF velocities
    dq = model.trunkFootIKVel(
        trunkPosVel, 
        Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel), 
        footPosVel,
        Leph::AxisDiffToAngularDiff(footAxis, footAxisVel));
    //Compute DOF accelerations
    ddq = model.trunkFootIKAcc(
        dq,
        trunkPosVel, 
        Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisVel), 
        footPosVel,
        Leph::AxisDiffToAngularDiff(footAxis, footAxisVel), 
        trunkPosAcc, 
        Leph::AxisDiffToAngularDiff(trunkAxis, trunkAxisAcc), 
        footPosAcc,
        Leph::AxisDiffToAngularDiff(footAxis, footAxisAcc));

    return true;
}

void TrajectoriesDisplay(
    const Trajectories& traj, RobotType type)
{
    //Plot Cartesian trajectory
    Leph::Plot plot;
    for (double t=traj.min();t<=traj.max();t+=0.01) {
        plot.add(Leph::VectorLabel(
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
        plot.add(Leph::VectorLabel(
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
        plot.add(Leph::VectorLabel(
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
    Leph::ModelViewer viewer(1200, 900);
    Leph::HumanoidFixedModel model(type);
    Leph::Scheduling scheduling;
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
        }
        //Compute ZMP
        Eigen::Vector3d zmp = model.zeroMomentPoint("origin", dq, ddq);
        zmp.z() = 0.0;
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
        torques(model.get().getDOFIndex("base_x")) = 0.0;
        torques(model.get().getDOFIndex("base_y")) = 0.0;
        torques(model.get().getDOFIndex("base_z")) = 0.0;
        torques(model.get().getDOFIndex("base_yaw")) = 0.0;
        torques(model.get().getDOFIndex("base_pitch")) = 0.0;
        torques(model.get().getDOFIndex("base_roll")) = 0.0;
        //Display ZMP and trunk/foot trajectory
        viewer.addTrackedPoint(
            model.get().position("trunk", "origin"), 
            Leph::ModelViewer::Purple);
        viewer.addTrackedPoint(
            model.get().position("right_foot_tip", "origin"), 
            Leph::ModelViewer::Cyan);
        viewer.addTrackedPoint(
            zmp, Leph::ModelViewer::Yellow);
        //Model display
        Leph::ModelDraw(model.get(), viewer);
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
            plot.add(Leph::VectorLabel(
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
    plot.plot("t", "zmp_x").plot("t", "zmp_y").render();
}

double DefaultCheckState(
    const Eigen::Vector3d& trunkPos,
    const Eigen::Vector3d& trunkAxis,
    const Eigen::Vector3d& footPos,
    const Eigen::Vector3d& footAxis)
{
    double cost = 0.0;
    if (trunkPos.z() < 0.0) {
        cost += 1000.0 - 1000.0*trunkPos.z();
    }
    if (trunkAxis.norm() >= M_PI/2.0) {
        cost += 1000.0 + 1000.0*(trunkAxis.norm() - M_PI/2.0);
    }
    if (footPos.y() > -2.0*0.045) {
        cost += 1000.0 + 1000.0*(footPos.y() + 2.0*0.045);
    }
    if (footPos.z() < 0.0) {
        cost += 1000.0 - 1000.0*footPos.z();
    }
    if (footAxis.norm() >= M_PI/2.0) {
        cost += 1000.0 + 1000.0*(footAxis.norm() - M_PI/2.0);
    }

    return cost;
}

double DefaultCheckDOF(const Leph::HumanoidFixedModel& modelFixed)
{
    double cost = 0.0;
    const HumanoidModel& model = modelFixed.get();
    if (fabs(model.getDOF("left_hip_yaw")) > M_PI/3.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("left_hip_yaw")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("left_hip_roll")) > M_PI/3.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("left_hip_roll")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("left_hip_pitch")) > M_PI/2.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("left_hip_pitch")) - M_PI/2.0);
    }
    if (fabs(model.getDOF("left_knee")) > 3.0*M_PI/2.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("left_knee")) - 3.0*M_PI/2.0);
    }
    if (fabs(model.getDOF("left_ankle_pitch")) > M_PI/2.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("left_ankle_pitch")) - M_PI/2.0);
    }
    if (fabs(model.getDOF("left_ankle_roll")) > M_PI/3.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("left_ankle_roll")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_hip_yaw")) > M_PI/3.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("right_hip_yaw")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_hip_roll")) > M_PI/3.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("right_hip_roll")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_hip_pitch")) > M_PI/2.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("right_hip_pitch")) - M_PI/2.0);
    }
    if (fabs(model.getDOF("right_knee")) > 3.0*M_PI/2.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("right_knee")) - 3.0*M_PI/2.0);
    }
    if (fabs(model.getDOF("right_ankle_pitch")) > M_PI/2.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("right_ankle_pitch")) - M_PI/2.0);
    }
    if (fabs(model.getDOF("right_ankle_roll")) > M_PI/3.0) {
        cost += 1000.0 + 
            1000.0*(fabs(model.getDOF("right_ankle_roll")) - M_PI/3.0);
    }

    return cost;
}

}

