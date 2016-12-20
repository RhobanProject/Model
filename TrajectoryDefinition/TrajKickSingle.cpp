#include "TrajectoryDefinition/TrajKickSingle.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"

namespace Leph {

Eigen::VectorXd TrajKickSingle::initialParameters(
    TrajectoryParameters& trajParams)
{
    //Total time length
    trajParams.set("time_length", false) = 3.0;
    //time ratio for control points
    trajParams.set("time_ratio_middle1", true) = 0.4;
    trajParams.set("time_ratio_contact", true) = 0.5;
    trajParams.set("time_ratio_middle2", true) = 0.6;
    
    //Position at contact
    trajParams.set("contact_pos_trunk_pos_x",  true)  = trajParams.get("static_single_pos_trunk_pos_x");
    trajParams.set("contact_pos_trunk_pos_y",  true)  = trajParams.get("static_single_pos_trunk_pos_y");
    trajParams.set("contact_pos_trunk_pos_z",  false) = trajParams.get("static_single_pos_trunk_pos_z");
    trajParams.set("contact_pos_trunk_axis_x", true)  = trajParams.get("static_single_pos_trunk_axis_x");
    trajParams.set("contact_pos_trunk_axis_y", true)  = trajParams.get("static_single_pos_trunk_axis_y");
    trajParams.set("contact_pos_trunk_axis_z", true)  = trajParams.get("static_single_pos_trunk_axis_z");
    trajParams.set("contact_pos_foot_pos_x",   false) = 0.01;
    trajParams.set("contact_pos_foot_pos_y",   false) = -0.11;
    trajParams.set("contact_pos_foot_pos_z",   false) = 0.07;
    trajParams.set("contact_pos_foot_axis_x",  false) = 0.0;
    trajParams.set("contact_pos_foot_axis_y",  false) = 0.0;
    trajParams.set("contact_pos_foot_axis_z",  false) = 0.0;
    //Velocity at contact
    trajParams.set("contact_vel_trunk_pos_x",  true)  = 0.0;
    trajParams.set("contact_vel_trunk_pos_y",  true)  = 0.0;
    trajParams.set("contact_vel_trunk_pos_z",  false) = 0.0;
    trajParams.set("contact_vel_trunk_axis_x", true)  = 0.0;
    trajParams.set("contact_vel_trunk_axis_y", true)  = 0.0;
    trajParams.set("contact_vel_trunk_axis_z", true)  = 0.0;
    trajParams.set("contact_vel_foot_pos_x",   false) = 1.0;
    trajParams.set("contact_vel_foot_pos_y",   false) = 0.0;
    trajParams.set("contact_vel_foot_pos_z",   false) = 0.0;
    //Acceleration at contact
    trajParams.set("contact_acc_trunk_pos_x",  true)  = 0.0;
    trajParams.set("contact_acc_trunk_pos_y",  true)  = 0.0;
    trajParams.set("contact_acc_trunk_pos_z",  false) = 0.0;
    trajParams.set("contact_acc_trunk_axis_x", true)  = 0.0;
    trajParams.set("contact_acc_trunk_axis_y", true)  = 0.0;
    trajParams.set("contact_acc_trunk_axis_z", true)  = 0.0;
    trajParams.set("contact_acc_foot_pos_x",   true)  = 0.0;
    trajParams.set("contact_acc_foot_pos_y",   false) = 0.0;
    trajParams.set("contact_acc_foot_pos_z",   true)  = 0.0;

    //Position at middle1
    trajParams.set("middle1_pos_trunk_pos_x",  true)  = trajParams.get("static_single_pos_trunk_pos_x");
    trajParams.set("middle1_pos_trunk_pos_y",  true)  = trajParams.get("static_single_pos_trunk_pos_y");
    trajParams.set("middle1_pos_trunk_pos_z",  false) = trajParams.get("static_single_pos_trunk_pos_z");
    trajParams.set("middle1_pos_trunk_axis_x", true)  = trajParams.get("static_single_pos_trunk_axis_x");
    trajParams.set("middle1_pos_trunk_axis_y", true)  = trajParams.get("static_single_pos_trunk_axis_y");
    trajParams.set("middle1_pos_trunk_axis_z", true)  = trajParams.get("static_single_pos_trunk_axis_z");
    trajParams.set("middle1_pos_foot_pos_x",   true)  = trajParams.get("static_single_pos_foot_pos_x");
    trajParams.set("middle1_pos_foot_pos_y",   false) = trajParams.get("contact_pos_foot_pos_y");
    trajParams.set("middle1_pos_foot_pos_z",   true)  = trajParams.get("static_single_pos_foot_pos_z");
    trajParams.set("middle1_pos_foot_axis_x",  false) = 0.0;
    trajParams.set("middle1_pos_foot_axis_y",  false) = 0.0;
    trajParams.set("middle1_pos_foot_axis_z",  false) = 0.0;
    //Velocity at middle1
    trajParams.set("middle1_vel_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle1_vel_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle1_vel_trunk_pos_z",  false) = 0.0;
    trajParams.set("middle1_vel_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle1_vel_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle1_vel_trunk_axis_z", true)  = 0.0;
    trajParams.set("middle1_vel_foot_pos_x",   false) = 0.0;
    trajParams.set("middle1_vel_foot_pos_y",   false) = 0.0;
    trajParams.set("middle1_vel_foot_pos_z",   false) = 0.0;
    //Acceleration at middle1
    trajParams.set("middle1_acc_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle1_acc_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle1_acc_trunk_pos_z",  false) = 0.0;
    trajParams.set("middle1_acc_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle1_acc_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle1_acc_trunk_axis_z", true)  = 0.0;
    trajParams.set("middle1_acc_foot_pos_x",   true)  = 0.0;
    trajParams.set("middle1_acc_foot_pos_y",   false) = 0.0;
    trajParams.set("middle1_acc_foot_pos_z",   true)  = 0.0;
    
    //Position at middle2
    trajParams.set("middle2_pos_trunk_pos_x",  true)  = trajParams.get("static_single_pos_trunk_pos_x");
    trajParams.set("middle2_pos_trunk_pos_y",  true)  = trajParams.get("static_single_pos_trunk_pos_y");
    trajParams.set("middle2_pos_trunk_pos_z",  false) = trajParams.get("static_single_pos_trunk_pos_z");
    trajParams.set("middle2_pos_trunk_axis_x", true)  = trajParams.get("static_single_pos_trunk_axis_x");
    trajParams.set("middle2_pos_trunk_axis_y", true)  = trajParams.get("static_single_pos_trunk_axis_y");
    trajParams.set("middle2_pos_trunk_axis_z", true)  = trajParams.get("static_single_pos_trunk_axis_z");
    trajParams.set("middle2_pos_foot_pos_x",   true)  = trajParams.get("static_single_pos_foot_pos_x");
    trajParams.set("middle2_pos_foot_pos_y",   false) = trajParams.get("contact_pos_foot_pos_y");
    trajParams.set("middle2_pos_foot_pos_z",   true)  = trajParams.get("static_single_pos_foot_pos_z");
    trajParams.set("middle2_pos_foot_axis_x",  false) = 0.0;
    trajParams.set("middle2_pos_foot_axis_y",  false) = 0.0;
    trajParams.set("middle2_pos_foot_axis_z",  false) = 0.0;
    //Velocity at middle2
    trajParams.set("middle2_vel_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle2_vel_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle2_vel_trunk_pos_z",  false) = 0.0;
    trajParams.set("middle2_vel_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle2_vel_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle2_vel_trunk_axis_z", true)  = 0.0;
    trajParams.set("middle2_vel_foot_pos_x",   false) = 0.0;
    trajParams.set("middle2_vel_foot_pos_y",   false) = 0.0;
    trajParams.set("middle2_vel_foot_pos_z",   false) = 0.0;
    //Acceleration at middle2
    trajParams.set("middle2_acc_trunk_pos_x",  true)  = 0.0;
    trajParams.set("middle2_acc_trunk_pos_y",  true)  = 0.0;
    trajParams.set("middle2_acc_trunk_pos_z",  false) = 0.0;
    trajParams.set("middle2_acc_trunk_axis_x", true)  = 0.0;
    trajParams.set("middle2_acc_trunk_axis_y", true)  = 0.0;
    trajParams.set("middle2_acc_trunk_axis_z", true)  = 0.0;
    trajParams.set("middle2_acc_foot_pos_x",   true)  = 0.0;
    trajParams.set("middle2_acc_foot_pos_y",   false) = 0.0;
    trajParams.set("middle2_acc_foot_pos_z",   true)  = 0.0;
    
    return trajParams.buildVector();
}

TrajectoryGeneration::GenerationFunc TrajKickSingle::funcGeneration(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](const Eigen::VectorXd& params) -> Leph::Trajectories {
        //Retrieve timing parameters
        double endTime = trajParams.get("time_length", params);
        double middle1Time = trajParams.get("time_ratio_middle1", params)*endTime;
        double contactTime = trajParams.get("time_ratio_contact", params)*endTime;
        double middle2Time = trajParams.get("time_ratio_middle2", params)*endTime;
        
        //Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        //Support phase (single support for kick)
        traj.get("is_double_support").addPoint(0.0, 0.0);
        traj.get("is_double_support").addPoint(endTime, 0.0);
        //Support foot 
        traj.get("is_left_support_foot").addPoint(0.0, 1.0);
        traj.get("is_left_support_foot").addPoint(endTime, 1.0);

        //Starting in static single support pose
        trajParams.trajectoriesAssign(
            traj, 0.0, "static_single", params);
        //Pre Kick middle
        trajParams.trajectoriesAssign(
            traj, middle1Time, "middle1", params);
        //Kick contact
        trajParams.trajectoriesAssign(
            traj, contactTime, "contact", params);
        //Post Kick middle
        trajParams.trajectoriesAssign(
            traj, middle2Time, "middle2", params);
        //Ending in single support pose
        trajParams.trajectoriesAssign(
            traj, endTime, "static_single", params);

        return traj;
    };
}

TrajectoryGeneration::CheckParamsFunc TrajKickSingle::funcCheckParams(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](const Eigen::VectorXd& params) -> double {
        //Retrieve timing parameters
        double middle1Ratio = trajParams.get("time_ratio_middle1", params);
        double contactRatio = trajParams.get("time_ratio_contact", params);
        double middle2Ratio = trajParams.get("time_ratio_middle2", params);
        //Check support ratio bound
        if (middle1Ratio <= 0.1) {
            return  1000.0 - 1000.0*middle1Ratio;
        }
        if (middle1Ratio >= 0.9) {
            return  1000.0 + 1000.0*(middle1Ratio - 1.0);
        }
        if (contactRatio <= 0.1) {
            return  1000.0 - 1000.0*contactRatio;
        }
        if (contactRatio >= 0.9) {
            return  1000.0 + 1000.0*(contactRatio - 1.0);
        }
        if (middle2Ratio <= 0.1) {
            return  1000.0 - 1000.0*middle2Ratio;
        }
        if (middle2Ratio >= 0.9) {
            return  1000.0 + 1000.0*(middle2Ratio - 1.0);
        }
        if (middle1Ratio + 0.05 > contactRatio) {
            return 1000.0 - 1000.0*(contactRatio-middle1Ratio-0.05);
        }
        if (contactRatio + 0.05 > middle2Ratio) {
            return 1000.0 - 1000.0*(middle2Ratio-contactRatio-0.05);
        }
        return 0.0;
    };
}

TrajectoryGeneration::CheckStateFunc TrajKickSingle::funcCheckState(
    const TrajectoryParameters& trajParams)
{
    return Leph::DefaultCheckState;
}

TrajectoryGeneration::CheckDOFFunc TrajKickSingle::funcCheckDOF(
    const TrajectoryParameters& trajParams)
{
    return Leph::DefaultCheckDOF;
}

TrajectoryGeneration::ScoreFunc TrajKickSingle::funcScore(
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
        (void)isDoubleSupport;
        (void)supportFoot;
        
        double cost = 0.0;
        
        //Init data
        if (data.size() == 0) {
            data.push_back(0.0);
            data.push_back(0.0);
            data.push_back(0.0);
        }

        //ZMP
        Eigen::Vector3d zmp = model.zeroMomentPointFromTorques("origin", torques);
        zmp.z() = 0.0;
        cost += zmp.norm();
        //Max ZMP
        if (data[0] < zmp.lpNorm<Eigen::Infinity>()) {
            data[0] = zmp.lpNorm<Eigen::Infinity>();
        }

        //Support torque yaw
        double torqueSupportYaw = fabs(torques(model.get().getDOFIndex("base_yaw")));
        //Max support yaw
        if (data[2] < torqueSupportYaw) {
            data[2] = torqueSupportYaw;
        }
        
        //Torques
        Eigen::VectorXd tmpTorques = torques;
        tmpTorques(model.get().getDOFIndex("base_x")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_y")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_z")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_yaw")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_pitch")) = 0.0;
        tmpTorques(model.get().getDOFIndex("base_roll")) = 0.0;
        cost += 0.01*tmpTorques.norm();
        
        //Voltage
        Leph::JointModel jointModel;
        for (const std::string& name : Leph::NamesDOF) {
            size_t index = model.get().getDOFIndex(name);
            double volt = jointModel.computeElectricTension(
                dq(index), ddq(index), torques(index));
            //Maximum voltage
            if (data[1] < volt) {
                data[1] = volt;
            }
        }

        return cost;
    };
}

TrajectoryGeneration::EndScoreFunc TrajKickSingle::funcEndScore(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](
        const Eigen::VectorXd& params,
        const Leph::Trajectories& traj, 
        double score,
        std::vector<double>& data,
        bool verbose) -> double {
        (void)params;
        (void)traj;
        if (verbose) {
            std::cout << "SumTorque=" << score 
                << " MaxZMP=" << data[0] 
                << " MaxVolt=" << data[1]
                << " MaxTorqueYaw=" << data[2] 
                << std::endl;
        }
        double cost = 0.0;
        if (data[2] > 0.2) {
            cost += 10.0 + 10.0*data[2];
            if (verbose) {
                std::cout << "TorqueYawBound=" << cost << std::endl;
            }
        }
        if (fabs(data[1]) > 0.75*12.0) {
            cost += 10.0 + 10.0*data[1];
            if (verbose) {
                std::cout << "VoltBound=" << cost << std::endl;
            }
        } else {
            cost += 150.0*data[0] + 0.2*data[1] + 2.0*data[2];
            if (verbose) {
                std::cout << "ZMPCost=" << 150.0*data[0] 
                    << " VoltCost=" << 0.2*data[1]
                    << " TorqueYawCost=" << 2.0*data[2] 
                    << std::endl;
            }
        }
        if (verbose) {
            std::cout 
                << "--> "
                << "IterationCost=" << score 
                << " EndCost=" << cost 
                << " Total=" << score + cost 
                << std::endl;
        }
        return cost;
    };
}

}

