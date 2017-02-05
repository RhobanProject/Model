#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
#include "Utils/FileEigen.h"
#include "Model/NamesModel.h"

namespace Leph {

TrajectoryParameters DefaultTrajParameters()
{
    //Default trajectory parameter initialization
    //(No Optimized)
    TrajectoryParameters parameters;
    //CMA-ES parameters
    parameters.add("cmaes_max_iterations", 1000.0);
    parameters.add("cmaes_restarts", 3.0);
    parameters.add("cmaes_lambda", 10.0);
    parameters.add("cmaes_sigma", -1.0);
    parameters.add("cmaes_elitism", 1.0);
    //Fitness maximum torque yaw
    parameters.add("fitness_max_torque_yaw", 1.5);
    //Fitness maximum voltage ratio
    parameters.add("fitness_max_volt_ratio", 2.0);
    //Double support static position
    parameters.add("static_double_pos_trunk_pos_x", 0.009816280388);
    parameters.add("static_double_pos_trunk_pos_y", -0.07149996496);
    parameters.add("static_double_pos_trunk_pos_z", 0.2786133349);
    parameters.add("static_double_pos_trunk_axis_x", 0.0);
    parameters.add("static_double_pos_trunk_axis_y", 0.1919862181);
    parameters.add("static_double_pos_trunk_axis_z", 0.0);
    parameters.add("static_double_pos_foot_pos_x", 0.0);
    parameters.add("static_double_pos_foot_pos_y", -0.1429999995);
    parameters.add("static_double_pos_foot_pos_z", 0.0);
    parameters.add("static_double_pos_foot_axis_x", 0.0);
    parameters.add("static_double_pos_foot_axis_y", 0.0);
    parameters.add("static_double_pos_foot_axis_z", 0.0);
    //Single support static position
    parameters.add("static_single_pos_trunk_pos_x", -0.00557785331559037);
    parameters.add("static_single_pos_trunk_pos_y", -0.0115849568418458);
    parameters.add("static_single_pos_trunk_pos_z", 0.285);
    parameters.add("static_single_pos_trunk_axis_x", -0.672036398746933);
    parameters.add("static_single_pos_trunk_axis_y", 0.0743358280850477);
    parameters.add("static_single_pos_trunk_axis_z", 0.0028323027017884);
    parameters.add("static_single_pos_foot_pos_x", 0.0208647084129351);
    parameters.add("static_single_pos_foot_pos_y", -0.095);
    parameters.add("static_single_pos_foot_pos_z", 0.0591693358237435);
    parameters.add("static_single_pos_foot_axis_x", 0.0);
    parameters.add("static_single_pos_foot_axis_y", 0.0);
    parameters.add("static_single_pos_foot_axis_z", 0.0);

    return parameters;
}

TrajectoryGeneration::ScoreFunc DefaultFuncScore(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](
        double t,
        HumanoidFixedModel& model,
        const std::map<std::string, JointModel>& joints,
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        bool isDoubleSupport,
        HumanoidFixedModel::SupportFoot supportFoot,
        std::vector<double>& data) -> double 
    {
        (void)t;

        //Init data
        if (data.size() == 0) {
            //[0] Count summed
            data.push_back(0.0);
            //[1] Sum voltage
            data.push_back(0.0);
            //[2] Max ZMP
            data.push_back(0.0);
            //[3] Max voltage overload
            data.push_back(0.0);
            //[4] Max torque yaw
            data.push_back(0.0);
        }

        //ZMP if single support
        if (!isDoubleSupport) {
            Eigen::Vector3d zmp;
            if (supportFoot == HumanoidFixedModel::LeftSupportFoot) {
                zmp = model.zeroMomentPointFromTorques(
                    "left_foot_tip", torques);
            } else {
                zmp = model.zeroMomentPointFromTorques(
                    "right_foot_tip", torques);
            }
            zmp.z() = 0.0;
            //Max ZMP
            if (data[2] < zmp.lpNorm<Eigen::Infinity>()) {
                data[2] = zmp.lpNorm<Eigen::Infinity>();
            }
        }
        
        //Voltage
        for (const std::string& name : NamesDOF) {
            size_t index = model.get().getDOFIndex(name);
            double volt = fabs(joints.at(name).computeElectricTension(
                dq(index), ddq(index), torques(index)));
            double maxVolt = trajParams.get("fitness_max_volt_ratio")
                * fabs(joints.at(name).getMaxVoltage());
            //Maximum voltage overload
            if (volt > maxVolt && data[3] < (volt-maxVolt)) {
                data[3] = volt-maxVolt;
            }
            data[0] += 1.0;
            data[1] += volt;
        }

        //Support torque yaw if single support
        if (!isDoubleSupport) {
            double torqueSupportYaw = fabs(
                torques(model.get().getDOFIndex("base_yaw")));
            //Max support yaw
            if (data[4] < torqueSupportYaw) {
                data[4] = torqueSupportYaw;
            }
        }
        
        return 0.0;
    };
}

TrajectoryGeneration::EndScoreFunc DefaultFuncEndScore(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](
        const Eigen::VectorXd& params,
        const Trajectories& traj, 
        double score,
        std::vector<double>& data,
        bool verbose) -> double {
        (void)params;
        (void)traj;
        //Skip if the first iteration
        //has not ended (data not initialize)
        if (data.size() != 5) {
            return 0.0;
        }
        //Verbose
        if (verbose) {
            std::cout 
                << "MeanVolt=" << data[1]/data[0] 
                << " MaxZMP=" << data[2] 
                << " MaxVoltOverload=" << data[3]
                << " MaxTorqueYaw=" << data[4] 
                << std::endl;
        }
        //Penalize high motor voltage overload
        double costVoltOverload = data[3];
        //Penalize motor mean voltage
        double costVoltMean = (data[1]/data[0])/5.0;
        //Penalize max ZMP
        double costZMP = data[2]/0.02;
        //Penalize high yaw support torque
        double costTorqueYaw = 0.0;
        if (data[4] > trajParams.get("fitness_max_torque_yaw")) {
            costTorqueYaw = 
                data[4]/trajParams.get("fitness_max_torque_yaw");
        }
        if (verbose) {
            std::cout 
                << "MeanVoltCost=" << costVoltMean
                << " MaxZMPCost=" << costZMP
                << " MaxVoltOverloadCost=" << costVoltOverload
                << " MaxTorqueYawCost=" << costTorqueYaw
                << std::endl;
        }
        return 
            costVoltMean + 
            costZMP + 
            costVoltOverload + 
            costTorqueYaw;
    };
}

TrajectoryGeneration::SaveFunc DefaultFuncSave(
    const TrajectoryParameters& trajParams)
{
    return [&trajParams](
        const std::string& filename,
        const Trajectories& traj,
        const Eigen::VectorXd& params) -> void 
    {
        if (filename != "") {
            traj.exportData(filename + ".splines");
            std::cout << "****** Saving Trajectories to: " 
                << filename + ".splines" << std::endl;
            trajParams.exportData(filename + ".params", params);
            std::cout << "****** Saving Parameters to: " 
                << filename + ".params" << std::endl;
        }
    };
}

}

