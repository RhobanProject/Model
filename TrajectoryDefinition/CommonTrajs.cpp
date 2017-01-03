#include "TrajectoryDefinition/CommonTrajs.h"
#include "Model/JointModel.hpp"
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
    //Fitness maximum torque yaw
    parameters.add("fitness_max_torque_yaw", 1.0);
    //Fitness maximum voltage ratio
    parameters.add("fitness_max_volt_ratio", 1.0);
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
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        bool isDoubleSupport,
        HumanoidFixedModel::SupportFoot supportFoot,
        std::vector<double>& data) -> double 
    {
        (void)t;
        double cost = 0.0;

        //Init data
        if (data.size() == 0) {
            //Max ZMP
            data.push_back(0.0);
            //Max voltage
            data.push_back(0.0);
            //Max torque yaw
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
            if (data[0] < zmp.lpNorm<Eigen::Infinity>()) {
                data[0] = zmp.lpNorm<Eigen::Infinity>();
            }
        }
        
        //Voltage
        JointModel jointModel;
        for (const std::string& name : NamesDOF) {
            size_t index = model.get().getDOFIndex(name);
            double volt = fabs(jointModel.computeElectricTension(
                dq(index), ddq(index), torques(index)));
            //Maximum voltage
            if (data[1] < volt) {
                data[1] = volt;
            }
            cost += 0.01*volt/NamesDOF.size();
        }

        //Support torque yaw if single support
        if (!isDoubleSupport) {
            double torqueSupportYaw = fabs(
                torques(model.get().getDOFIndex("base_yaw")));
            //Max support yaw
            if (data[2] < torqueSupportYaw) {
                data[2] = torqueSupportYaw;
            }
        }
        
        return cost;
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
        if (verbose) {
            std::cout 
                << "MeanVolt=" << score 
                << " MaxZMP=" << data[0] 
                << " MaxVolt=" << data[1]
                << " MaxTorqueYaw=" << data[2] 
                << std::endl;
        }
        double cost = 0.0;
        JointModel jointModel;
        if (data[1] > trajParams.get("fitness_max_volt_ratio")
            *jointModel.getMaxVoltage()
        ) {
            double tmpCost = 10.0 + 10.0*data[1];
            cost += tmpCost;
            if (verbose) {
                std::cout << "VoltBound=" << tmpCost << std::endl;
            }
        } 
        if (data[2] > trajParams.get("fitness_max_torque_yaw")) {
            double tmpCost = 10.0 + 10.0*data[2];
            cost += tmpCost;
            if (verbose) {
                std::cout << "TorqueYawBound=" << tmpCost << std::endl;
            }
        }
        cost += 150.0*data[0] + 0.2*data[1] + 1.0*data[2];
        if (verbose) {
            std::cout 
                << "ZMPCost=" << 150.0*data[0] 
                << " VoltCost=" << 0.2*data[1]
                << " TorqueYawCost=" << 1.0*data[2] 
                << std::endl;
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

