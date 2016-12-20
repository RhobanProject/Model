#include "TrajectoryDefinition/TrajLegLift.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

namespace Leph {

Eigen::VectorXd TrajLegLift::initialParameters(
    TrajectoryParameters& trajParams)
{
    //Total time length
    trajParams.set("time_length", false) = 3.0;
    //time ratio for control points
    trajParams.set("time_ratio_middle1", true) = 0.3;
    trajParams.set("time_ratio_swap", true) = 0.5;
    trajParams.set("time_ratio_middle2", true) = 0.7;
    
    //Position at swap
    trajParams.set("swap_pos_trunk_pos_x",  true)  = 
        0.7*trajParams.get("static_double_pos_trunk_pos_x") + 0.3*trajParams.get("static_single_pos_trunk_pos_x");
    trajParams.set("swap_pos_trunk_pos_y",  true)  = 
        0.7*trajParams.get("static_double_pos_trunk_pos_y") + 0.3*trajParams.get("static_single_pos_trunk_pos_y");
    trajParams.set("swap_pos_trunk_pos_z",  true)  = 
        0.7*trajParams.get("static_double_pos_trunk_pos_z") + 0.3*trajParams.get("static_single_pos_trunk_pos_z");
    trajParams.set("swap_pos_trunk_axis_x", true)  = 
        0.7*trajParams.get("static_double_pos_trunk_axis_x") + 0.3*trajParams.get("static_single_pos_trunk_axis_x");
    trajParams.set("swap_pos_trunk_axis_y", true)  = 
        0.7*trajParams.get("static_double_pos_trunk_axis_y") + 0.3*trajParams.get("static_single_pos_trunk_axis_y");
    trajParams.set("swap_pos_trunk_axis_z", true)  = 
        0.7*trajParams.get("static_double_pos_trunk_axis_z") + 0.3*trajParams.get("static_single_pos_trunk_axis_z");
    trajParams.set("swap_pos_foot_pos_x",   false) = trajParams.get("static_double_pos_foot_pos_x");
    trajParams.set("swap_pos_foot_pos_y",   false) = trajParams.get("static_double_pos_foot_pos_y");
    trajParams.set("swap_pos_foot_pos_z",   false) = trajParams.get("static_double_pos_foot_pos_z");
    //Velocity at swap
    trajParams.set("swap_vel_trunk_pos_x",  true)  = 0.0;
    trajParams.set("swap_vel_trunk_pos_y",  true)  = 0.0;
    trajParams.set("swap_vel_trunk_pos_z",  true)  = 0.0;
    trajParams.set("swap_vel_trunk_axis_x", true)  = 0.0;
    trajParams.set("swap_vel_trunk_axis_y", true)  = 0.0;
    trajParams.set("swap_vel_trunk_axis_z", true)  = 0.0;
    //Acceleration at swap
    trajParams.set("swap_acc_trunk_pos_x",  true)  = 0.0;
    trajParams.set("swap_acc_trunk_pos_y",  true)  = 0.0;
    trajParams.set("swap_acc_trunk_pos_z",  true)  = 0.0;
    trajParams.set("swap_acc_trunk_axis_x", true)  = 0.0;
    trajParams.set("swap_acc_trunk_axis_y", true)  = 0.0;
    trajParams.set("swap_acc_trunk_axis_z", true)  = 0.0;

    //Position at middle1
    trajParams.set("middle1_pos_trunk_pos_x",  true)  = 
        0.75*trajParams.get("static_double_pos_trunk_pos_x") + 0.25*trajParams.get("static_single_pos_trunk_pos_x");
    trajParams.set("middle1_pos_trunk_pos_y",  true)  = 
        0.75*trajParams.get("static_double_pos_trunk_pos_y") + 0.25*trajParams.get("static_single_pos_trunk_pos_y");
    trajParams.set("middle1_pos_trunk_pos_z",  true)  = 
        0.75*trajParams.get("static_double_pos_trunk_pos_z") + 0.25*trajParams.get("static_single_pos_trunk_pos_z");
    trajParams.set("middle1_pos_trunk_axis_x", true)  = 
        0.75*trajParams.get("static_double_pos_trunk_axis_x") + 0.25*trajParams.get("static_single_pos_trunk_axis_x");
    trajParams.set("middle1_pos_trunk_axis_y", true)  = 
        0.75*trajParams.get("static_double_pos_trunk_axis_y") + 0.25*trajParams.get("static_single_pos_trunk_axis_y");
    trajParams.set("middle1_pos_trunk_axis_z", true)  = 
        0.75*trajParams.get("static_double_pos_trunk_axis_z") + 0.25*trajParams.get("static_single_pos_trunk_axis_z");
    trajParams.set("middle1_pos_foot_pos_x",   false) = trajParams.get("static_double_pos_foot_pos_x");
    trajParams.set("middle1_pos_foot_pos_y",   false) = trajParams.get("static_double_pos_foot_pos_y");
    trajParams.set("middle1_pos_foot_pos_z",   false) = trajParams.get("static_double_pos_foot_pos_z");
    //Velocity at middle1
    trajParams.set("middle1_vel_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle1_vel_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle1_vel_trunk_pos_z",  true)  = 0.0;
    trajParams.set("middle1_vel_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle1_vel_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle1_vel_trunk_axis_z", true)  = 0.0;
    //Acceleration at middle1
    trajParams.set("middle1_acc_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle1_acc_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle1_acc_trunk_pos_z",  true)  = 0.0;
    trajParams.set("middle1_acc_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle1_acc_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle1_acc_trunk_axis_z", true)  = 0.0;
    
    //Position at middle2
    trajParams.set("middle2_pos_trunk_pos_x",  true)  = 
        0.5*trajParams.get("static_double_pos_trunk_pos_x") + 0.5*trajParams.get("static_single_pos_trunk_pos_x");
    trajParams.set("middle2_pos_trunk_pos_y",  true)  = 
        0.5*trajParams.get("static_double_pos_trunk_pos_y") + 0.5*trajParams.get("static_single_pos_trunk_pos_y");
    trajParams.set("middle2_pos_trunk_pos_z",  true)  = 
        0.5*trajParams.get("static_double_pos_trunk_pos_z") + 0.5*trajParams.get("static_single_pos_trunk_pos_z");
    trajParams.set("middle2_pos_trunk_axis_x", true)  = 
        0.5*trajParams.get("static_double_pos_trunk_axis_x") + 0.5*trajParams.get("static_single_pos_trunk_axis_x");
    trajParams.set("middle2_pos_trunk_axis_y", true)  = 
        0.5*trajParams.get("static_double_pos_trunk_axis_y") + 0.5*trajParams.get("static_single_pos_trunk_axis_y");
    trajParams.set("middle2_pos_trunk_axis_z", true)  = 
        0.5*trajParams.get("static_double_pos_trunk_axis_z") + 0.5*trajParams.get("static_single_pos_trunk_axis_z");
    trajParams.set("middle2_pos_foot_pos_x",   true)  = 
        0.5*trajParams.get("static_double_pos_foot_pos_x") + 0.5*trajParams.get("static_single_pos_foot_pos_x");
    trajParams.set("middle2_pos_foot_pos_y",   true)  = 
        0.5*trajParams.get("static_double_pos_foot_pos_y") + 0.5*trajParams.get("static_single_pos_foot_pos_y");
    trajParams.set("middle2_pos_foot_pos_z",   true)  = 
        0.5*trajParams.get("static_double_pos_foot_pos_z") + 0.5*trajParams.get("static_single_pos_foot_pos_z");
    //Velocity at middle2
    trajParams.set("middle2_vel_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle2_vel_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle2_vel_trunk_pos_z",  true)  = 0.0;
    trajParams.set("middle2_vel_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle2_vel_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle2_vel_trunk_axis_z", true)  = 0.0;
    trajParams.set("middle2_vel_foot_pos_x",   true)  = 0.0;
    trajParams.set("middle2_vel_foot_pos_y",   true)  = 0.0;
    trajParams.set("middle2_vel_foot_pos_z",   true)  = 0.0;
    //Acceleration at middle2
    trajParams.set("middle2_acc_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle2_acc_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle2_acc_trunk_pos_z",  true)  = 0.0;
    trajParams.set("middle2_acc_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle2_acc_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle2_acc_trunk_axis_z", true)  = 0.0;
    trajParams.set("middle2_acc_foot_pos_x",   true)  = 0.0;
    trajParams.set("middle2_acc_foot_pos_y",   true)  = 0.0;
    trajParams.set("middle2_acc_foot_pos_z",   true)  = 0.0;

    return trajParams.buildVector();
}

TrajectoryGeneration::GenerationFunc TrajLegLift::funcGeneration(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](const Eigen::VectorXd& params) -> Leph::Trajectories {
        //Retrieve timing parameters
        double endTime = trajParams.get("time_length", params);
        double middle1Time = trajParams.get("time_ratio_middle1", params)*endTime;
        double swapTime = trajParams.get("time_ratio_swap", params)*endTime;
        double middle2Time = trajParams.get("time_ratio_middle2", params)*endTime;
        
        //Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        //Support phase
        traj.get("is_double_support").addPoint(0.0, 1.0);
        traj.get("is_double_support").addPoint(swapTime, 1.0);
        traj.get("is_double_support").addPoint(swapTime, 0.0);
        traj.get("is_double_support").addPoint(endTime, 0.0);
        //Foot support
        traj.get("is_left_support_foot").addPoint(0.0, 1.0);
        traj.get("is_left_support_foot").addPoint(endTime, 1.0);

        //Starting in static double support pose
        trajParams.trajectoriesAssign(
            traj, 0.0, "static_double", params);
        //Pre swap time
        trajParams.trajectoriesAssign(
            traj, middle1Time, "middle1", params);
        //Support swap
        trajParams.trajectoriesAssign(
            traj, swapTime, "swap", params);
        //Post swap middle
        trajParams.trajectoriesAssign(
            traj, middle2Time, "middle2", params);
        //Ending in single support pose
        trajParams.trajectoriesAssign(
            traj, endTime, "static_single", params);

        return traj;
    };
}

TrajectoryGeneration::CheckParamsFunc TrajLegLift::funcCheckParams(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](const Eigen::VectorXd& params) -> double {
        //Retrieve timing parameters
        double middle1Ratio = trajParams.get("time_ratio_middle1", params);
        double swapRatio = trajParams.get("time_ratio_swap", params);
        double middle2Ratio = trajParams.get("time_ratio_middle2", params);
        //Check support ratio bound
        if (middle1Ratio <= 0.1) {
            return  1000.0 - 1000.0*middle1Ratio;
        }
        if (middle1Ratio >= 0.9) {
            return  1000.0 + 1000.0*(middle1Ratio - 1.0);
        }
        if (swapRatio <= 0.1) {
            return  1000.0 - 1000.0*swapRatio;
        }
        if (swapRatio >= 0.9) {
            return  1000.0 + 1000.0*(swapRatio - 1.0);
        }
        if (middle2Ratio <= 0.1) {
            return  1000.0 - 1000.0*middle2Ratio;
        }
        if (middle2Ratio >= 0.9) {
            return  1000.0 + 1000.0*(middle2Ratio - 1.0);
        }
        if (middle1Ratio + 0.05 > swapRatio) {
            return 1000.0 - 1000.0*(swapRatio-middle1Ratio-0.05);
        }
        if (swapRatio + 0.05 > middle2Ratio) {
            return 1000.0 - 1000.0*(middle2Ratio-swapRatio-0.05);
        }
        return 0.0;
    };
}

TrajectoryGeneration::CheckStateFunc TrajLegLift::funcCheckState(
    const TrajectoryParameters& trajParams)
{
    return Leph::DefaultCheckState;
}

TrajectoryGeneration::CheckDOFFunc TrajLegLift::funcCheckDOF(
    const TrajectoryParameters& trajParams)
{
    return Leph::DefaultCheckDOF;
}

TrajectoryGeneration::ScoreFunc TrajLegLift::funcScore(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](
        double t,
        Leph::HumanoidFixedModel& model,
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        bool isDoubleSupport,
        Leph::HumanoidFixedModel::SupportFoot supportFoot,
        std::vector<double>& data) -> double 
    {
        (void)t;
        (void)model;
        (void)dq;
        (void)ddq;

        Eigen::VectorXd tmpTorques = torques;
        tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;

        double cost = 0.0;
        cost += 0.01*tmpTorques.norm();
        if (data.size() == 0) {
            data.push_back(0.0);
        }
        if (data[0] < tmpTorques.lpNorm<Eigen::Infinity>()) {
            data[0] = tmpTorques.lpNorm<Eigen::Infinity>();
        }

        return cost;
    };
}

TrajectoryGeneration::EndScoreFunc TrajLegLift::funcEndScore(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](
        const Eigen::VectorXd& params,
        const Leph::Trajectories& traj, 
        double score,
        std::vector<double>& data,
        bool verbose) -> double 
    {
        (void)params;
        (void)traj;
        (void)score;
        (void)verbose;
        return data[0];
    };
}

}

