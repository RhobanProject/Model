#include <iostream>
#include <Eigen/Dense>
#include "Spline/SmoothSpline.hpp"
#include "Utils/time.h"
#include "Spline/SplineContainer.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"

/**
 * All Joint DOF names
 */
static std::vector<std::string> dofNames = {
    "left_ankle_pitch", "left_ankle_roll", "left_knee",
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
    "right_ankle_pitch", "right_ankle_roll", "right_knee",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw"
};

/**
 * Ending precompute optimal state
 */
static Eigen::Vector3d targetTrunkPos()
{
    return Eigen::Vector3d(-0.00557785331559037,  -0.0115849568418458, 0.28);
        //0.291527819747366);
}
static Eigen::Vector3d targetTrunkAxis()
{
    return Eigen::Vector3d(-0.672036398746933, 0.0743358280850477, 0.0028323027017884);
}
static Eigen::Vector3d targetFootPos()
{
    return Eigen::Vector3d(0.0208647084129351, -0.095, 0.0591693358237435);
        //-0.0900000062267158, 0.0591693358237435);
}

/**
 * Return initial state
 */
static Eigen::Vector3d initTrunkPos()
{
    return Eigen::Vector3d(0.0, -0.05, 0.20);
}
static Eigen::Vector3d initTrunkAxis()
{
    return Eigen::Vector3d(0.0, 0.0, 0.0);
}
static Eigen::Vector3d initFootPos()
{
    return Eigen::Vector3d(0.0, -0.1, 0.0);
}

void generateLegLift()
{
    //Initialize the generator
    Leph::TrajectoryGeneration generator(Leph::SigmabanModel);
    
    //Initial trajectory parameters
    generator.setInitialParameters([]() -> Eigen::VectorXd {
        Eigen::VectorXd params(43);
        //Double support ratio
        params(0) = 0.5;
        //Support swap trunk pos
        params(1) = 0.5*initTrunkPos().x() + 0.5*targetTrunkPos().x();
        params(2) = 0.5*initTrunkPos().y() + 0.5*targetTrunkPos().y();
        params(3) = 0.5*initTrunkPos().z() + 0.5*targetTrunkPos().z();
        //Support swap trunk vel
        params(4) = 0.0;
        params(5) = 0.0;
        params(6) = 0.0;
        //Support swap trunk axis
        params(7) = 0.5*initTrunkAxis().x() + 0.5*targetTrunkAxis().x();
        params(8) = 0.5*initTrunkAxis().y() + 0.5*targetTrunkAxis().y();
        params(9) = 0.5*initTrunkAxis().z() + 0.5*targetTrunkAxis().z();
        //Support swap trunk vel
        params(10) = 0.0;
        params(11) = 0.0;
        params(12) = 0.0;
        
        //Middle 1 trunk pos
        params(13) = 0.75*initTrunkPos().x() + 0.25*targetTrunkPos().x();
        params(14) = 0.75*initTrunkPos().y() + 0.25*targetTrunkPos().y();
        params(15) = 0.75*initTrunkPos().z() + 0.25*targetTrunkPos().z();
        //Middle 1 trunk vel
        params(16) = 0.0;
        params(17) = 0.0;
        params(18) = 0.0;
        //Middle 1 trunk axis
        params(19) = 0.75*initTrunkAxis().x() + 0.25*targetTrunkAxis().x();
        params(20) = 0.75*initTrunkAxis().y() + 0.25*targetTrunkAxis().y();
        params(21) = 0.75*initTrunkAxis().z() + 0.25*targetTrunkAxis().z();
        //Middle 1 trunk vel
        params(22) = 0.0;
        params(23) = 0.0;
        params(24) = 0.0;
        
        //Middle 2 trunk pos
        params(25) = 0.25*initTrunkPos().x() + 0.75*targetTrunkPos().x();
        params(26) = 0.25*initTrunkPos().y() + 0.75*targetTrunkPos().y();
        params(27) = 0.25*initTrunkPos().z() + 0.75*targetTrunkPos().z();
        //Middle 2 trunk vel
        params(28) = 0.0;
        params(29) = 0.0;
        params(30) = 0.0;
        //Middle 2 trunk axis
        params(31) = 0.25*initTrunkAxis().x() + 0.75*targetTrunkAxis().x();
        params(32) = 0.25*initTrunkAxis().y() + 0.75*targetTrunkAxis().y();
        params(33) = 0.25*initTrunkAxis().z() + 0.75*targetTrunkAxis().z();
        //Middle 2 trunk vel
        params(34) = 0.0;
        params(35) = 0.0;
        params(36) = 0.0;
        //Middle 2 foot pos
        params(37) = 0.5*initFootPos().x() + 0.5*targetFootPos().x();
        params(38) = 0.5*initFootPos().y() + 0.5*targetFootPos().y();
        params(39) = 0.5*initFootPos().z() + 0.5*targetFootPos().z();
        //Middle 2 foot vel
        params(40) = 0.0;
        params(41) = 0.0;
        params(42) = 0.0;

        return params;
    }());
    
    //Set Trajectory generation function
    generator.setTrajectoryGenerationFunc([](const Eigen::VectorXd& params) -> Leph::Trajectories {
        //Extract parameters
        double endTrajectoryTime = 3.0;
        double ratioSwapSupport = params(0);
        double swapSupportTime = ratioSwapSupport*endTrajectoryTime;
        Eigen::Vector3d trunkPosAtSwap = params.segment<3>(1);
        Eigen::Vector3d trunkPosVelAtSwap = params.segment<3>(4);
        Eigen::Vector3d trunkAngleAtSwap = params.segment<3>(7);
        Eigen::Vector3d trunkAngleVelAtSwap = params.segment<3>(10);
        Eigen::Vector3d footPosAtSwap = initFootPos();
        
        double middle1Time = 0.5*ratioSwapSupport*endTrajectoryTime;
        Eigen::Vector3d trunkPosAtMiddle1 = params.segment<3>(13);
        Eigen::Vector3d trunkPosVelAtMiddle1 = params.segment<3>(16);
        Eigen::Vector3d trunkAngleAtMiddle1 = params.segment<3>(19);
        Eigen::Vector3d trunkAngleVelAtMiddle1 = params.segment<3>(22);
        
        double middle2Time = (0.5*ratioSwapSupport + 0.5)*endTrajectoryTime;
        Eigen::Vector3d trunkPosAtMiddle2 = params.segment<3>(25);
        Eigen::Vector3d trunkPosVelAtMiddle2 = params.segment<3>(28);
        Eigen::Vector3d trunkAngleAtMiddle2 = params.segment<3>(31);
        Eigen::Vector3d trunkAngleVelAtMiddle2 = params.segment<3>(34);
        Eigen::Vector3d footPosAtMiddle2 = params.segment<3>(37);
        Eigen::Vector3d footPosVelAtMiddle2 = params.segment<3>(40);

        //Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        //Foot orientation
        traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

        //Foot support
        traj.get("is_left_support_foot").addPoint(0.0, 1.0, 0.0);
        traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0, 0.0);

        //Support phase
        traj.get("is_double_support").addPoint(0.0, 1.0, 0.0);
        traj.get("is_double_support").addPoint(swapSupportTime, 1.0, 0.0);
        traj.get("is_double_support").addPoint(swapSupportTime, 0.0, 0.0);
        traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);

        //Starting double support pose
        traj.get("trunk_pos_x").addPoint(0.0, initTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(0.0, initTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(0.0, initTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(0.0, initTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(0.0, initTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(0.0, initTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(0.0, initFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(0.0, initFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(0.0, initFootPos().z(), 0.0);

        //Double support phase middle
        traj.get("trunk_pos_x").addPoint(middle1Time, trunkPosAtMiddle1.x(), trunkPosVelAtMiddle1.x());
        traj.get("trunk_pos_y").addPoint(middle1Time, trunkPosAtMiddle1.y(), trunkPosVelAtMiddle1.y());
        traj.get("trunk_pos_z").addPoint(middle1Time, trunkPosAtMiddle1.z(), trunkPosVelAtMiddle1.z());
        traj.get("trunk_axis_x").addPoint(middle1Time, trunkAngleAtMiddle1.x(), trunkAngleVelAtMiddle1.x());
        traj.get("trunk_axis_y").addPoint(middle1Time, trunkAngleAtMiddle1.y(), trunkAngleVelAtMiddle1.y());
        traj.get("trunk_axis_z").addPoint(middle1Time, trunkAngleAtMiddle1.z(), trunkAngleVelAtMiddle1.z());
        
        //Double to single support phase
        traj.get("trunk_pos_x").addPoint(swapSupportTime, trunkPosAtSwap.x(), trunkPosVelAtSwap.x());
        traj.get("trunk_pos_y").addPoint(swapSupportTime, trunkPosAtSwap.y(), trunkPosVelAtSwap.y());
        traj.get("trunk_pos_z").addPoint(swapSupportTime, trunkPosAtSwap.z(), trunkPosVelAtSwap.z());
        traj.get("trunk_axis_x").addPoint(swapSupportTime, trunkAngleAtSwap.x(), trunkAngleVelAtSwap.x());
        traj.get("trunk_axis_y").addPoint(swapSupportTime, trunkAngleAtSwap.y(), trunkAngleVelAtSwap.y());
        traj.get("trunk_axis_z").addPoint(swapSupportTime, trunkAngleAtSwap.z(), trunkAngleVelAtSwap.z());
        traj.get("foot_pos_x").addPoint(swapSupportTime, footPosAtSwap.x(), 0.0);
        traj.get("foot_pos_y").addPoint(swapSupportTime, footPosAtSwap.y(), 0.0);
        traj.get("foot_pos_z").addPoint(swapSupportTime, footPosAtSwap.z(), 0.0);
        
        //Double support phase middle
        traj.get("trunk_pos_x").addPoint(middle2Time, trunkPosAtMiddle2.x(), trunkPosVelAtMiddle2.x());
        traj.get("trunk_pos_y").addPoint(middle2Time, trunkPosAtMiddle2.y(), trunkPosVelAtMiddle2.y());
        traj.get("trunk_pos_z").addPoint(middle2Time, trunkPosAtMiddle2.z(), trunkPosVelAtMiddle2.z());
        traj.get("trunk_axis_x").addPoint(middle2Time, trunkAngleAtMiddle2.x(), trunkAngleVelAtMiddle2.x());
        traj.get("trunk_axis_y").addPoint(middle2Time, trunkAngleAtMiddle2.y(), trunkAngleVelAtMiddle2.y());
        traj.get("trunk_axis_z").addPoint(middle2Time, trunkAngleAtMiddle2.z(), trunkAngleVelAtMiddle2.z());
        traj.get("foot_pos_x").addPoint(middle2Time, footPosAtMiddle2.x(), footPosVelAtMiddle2.x());
        traj.get("foot_pos_y").addPoint(middle2Time, footPosAtMiddle2.y(), footPosVelAtMiddle2.y());
        traj.get("foot_pos_z").addPoint(middle2Time, footPosAtMiddle2.z(), footPosVelAtMiddle2.z());
        
        //Ending single support pose
        traj.get("trunk_pos_x").addPoint(endTrajectoryTime, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(endTrajectoryTime, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(endTrajectoryTime, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(endTrajectoryTime, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(endTrajectoryTime, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(endTrajectoryTime, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(endTrajectoryTime, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(endTrajectoryTime, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(endTrajectoryTime, targetFootPos().z(), 0.0);

        return traj;
    });
    
    //Set parameters bound function
    generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
        //Check support ratio bound
        if (params(0) <= 0.1) {
            return  1000.0 - 1000.0*(params(0) - 0.1);
        }
        if (params(0) >= 0.9) {
            return  1000.0 + 1000.0*(params(0) - 0.9);
        }
        return 0.0;
    });
    //Set default Cartesian state and Joint DOF bounds
    generator.setCheckStateFunc(Leph::DefaultCheckState);
    generator.setCheckDOFFunc(Leph::DefaultCheckDOF);
    //Set trajectory scoring function
    generator.setScoreFunc([](
        double t,
        Leph::HumanoidFixedModel& model,
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        std::vector<double>& data) -> double {
        
        (void)t;
        (void)model;
        (void)dq;
        (void)ddq;
        double cost = 0.0;
        cost += 0.01*torques.norm();
        if (data.size() == 0) {
            data.push_back(0.0);
        }
        if (data[0] < torques.lpNorm<Eigen::Infinity>()) {
            data[0] = torques.lpNorm<Eigen::Infinity>();
        }

        return cost;
    });
    generator.setEndScoreFunc([](
        const Eigen::VectorXd& params,
        const Leph::Trajectories& traj,
        std::vector<double>& data) -> double {
        (void)params;
        (void)traj;
        return data[0];
    });
    
    //Display initial trajectory
    TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

    //Target filename
    std::string filename = "/tmp/trajLegLift_" + Leph::currentDate() + ".splines";
    //Run the CMA-ES optimization
    generator.runOptimization(1000, 5, filename, 20, -1.0);

    //Display found trajectory
    TrajectoriesDisplay(generator.bestTrajectories());
}

void generateKick()
{
    double KickPosX = 0.01;
    double KickPosY = -0.11;
    double KickPosZ = 0.07;

    //Initialize the generator
    Leph::TrajectoryGeneration generator(Leph::SigmabanModel);
    
    //Initial trajectory parameters
    generator.setInitialParameters([&KickPosX, &KickPosY, &KickPosZ]() -> Eigen::VectorXd {
        Eigen::VectorXd params(51);
        //Contact time ratio
        params(0) = 0.5;
        //Contact trunk pos
        params(1) = targetTrunkPos().x();
        params(2) = targetTrunkPos().y();
        //Contact trunk vel
        params(3) = 0.0;
        params(4) = 0.0;
        //Contact trunk axis
        params(5) = targetTrunkAxis().x();
        params(6) = targetTrunkAxis().y();
        params(7) = targetTrunkAxis().z();
        //Contact trunk vel
        params(8) = 0.0;
        params(9) = 0.0;
        params(10) = 0.0;
        
        //Middle 1 trunk pos
        params(11) = targetTrunkPos().x() - 0.01;
        params(12) = targetTrunkPos().y();
        //Middle 1 trunk axis
        params(13) = targetTrunkAxis().x();
        params(14) = targetTrunkAxis().y();
        params(15) = targetTrunkAxis().z();
        //Middle 1 foot pos
        params(16) = KickPosX + 0.01;
        params(17) = KickPosZ;
        //Middle 1 foot vel
        params(18) = 0.0;
        params(19) = 0.0;
        
        //Middle 2 trunk pos
        params(20) = targetTrunkPos().x() + 0.01;
        params(21) = targetTrunkPos().y();
        //Middle 2 trunk vel
        params(22) = 0.0;
        params(23) = 0.0;
        //Middle 2 trunk axis
        params(24) = targetTrunkAxis().x();
        params(25) = targetTrunkAxis().y();
        params(26) = targetTrunkAxis().z();
        //Middle 2 trunk vel
        params(27) = 0.0;
        params(28) = 0.0;
        params(29) = 0.0;
        //Middle 2 foot pos
        params(30) = KickPosX - 0.01;
        params(31) = KickPosZ;
        //Middle 2 foot vel
        params(32) = 0.0;
        params(33) = 0.0;
        
        //Contact/Middle1/Middle2 accelerations
        params(34) = 0.0;
        params(35) = 0.0;
        params(36) = 0.0;
        params(37) = 0.0;
        params(38) = 0.0;
        params(39) = 0.0;
        params(40) = 0.0;
        params(41) = 0.0;
        params(42) = 0.0;
        params(43) = 0.0;
        params(44) = 0.0;
        params(45) = 0.0;
        params(46) = 0.0;
        params(47) = 0.0;
        params(48) = 0.0;
        
        //Middle1/Middle2 time ratio
        params(49) = 0.3;
        params(50) = 0.7;
        
        return params;
    }());

    //Set Trajectory generation function
    generator.setTrajectoryGenerationFunc([&KickPosX, &KickPosY, &KickPosZ](const Eigen::VectorXd& params) -> Leph::Trajectories {
        double KickSpeed = 1.0;
        double endTrajectoryTime = 3.0;
        double ratioContactTime = params(0);

        double contactTime = ratioContactTime*endTrajectoryTime;
        Eigen::Vector3d trunkPosAtContact(params(1), params(2), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAtContact(params(3), params(4), 0.0);
        Eigen::Vector3d trunkAngleAtContact = params.segment<3>(5);
        Eigen::Vector3d trunkAngleVelAtContact = params.segment<3>(8);
        Eigen::Vector3d footPosAtContact(KickPosX, KickPosY, KickPosZ);
        Eigen::Vector3d footPosVelAtContact(KickSpeed, 0.0, 0.0);
        
        double middle1Time = params(49)*endTrajectoryTime;
        Eigen::Vector3d trunkPosAtMiddle1(params(11), params(12), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d trunkAngleAtMiddle1 = params.segment<3>(13);
        Eigen::Vector3d trunkAngleVelAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d footPosAtMiddle1(params(16), KickPosY, params(17));
        Eigen::Vector3d footPosVelAtMiddle1(params(18), 0.0, params(19));
        
        double middle2Time = params(50)*endTrajectoryTime;
        Eigen::Vector3d trunkPosAtMiddle2(params(20), params(21), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAtMiddle2(params(22), params(23), 0.0);
        Eigen::Vector3d trunkAngleAtMiddle2 = params.segment<3>(24);
        Eigen::Vector3d trunkAngleVelAtMiddle2 = params.segment<3>(27);
        Eigen::Vector3d footPosAtMiddle2(params(30), KickPosY, params(31));
        Eigen::Vector3d footPosVelAtMiddle2(params(32), 0.0, params(33));

        Eigen::Vector3d trunkPosAccAtContact(params(34), params(35), 0.0);
        Eigen::Vector3d trunkAngleAccAtContact(params(36), params(37), params(38));
        Eigen::Vector3d footPosAccAtContact(params(39), 0.0, 0.0);
        Eigen::Vector3d trunkPosAccAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d trunkAngleAccAtMiddle1(0.0, 0.0, 0.0);
        Eigen::Vector3d footPosAccAtMiddle1(params(40), 0.0, params(41));
        Eigen::Vector3d trunkPosAccAtMiddle2(params(42), params(43), 0.0);
        Eigen::Vector3d trunkAngleAccAtMiddle2(params(44), params(45), params(46));
        Eigen::Vector3d footPosAccAtMiddle2(params(47), 0.0, params(48));

        //Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        //Support phase (single support for kick)
        traj.get("is_double_support").addPoint(0.0, 0.0, 0.0);
        traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);
        //Support foot 
        traj.get("is_left_support_foot").addPoint(0.0, 1.0);
        traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0);

        //Starting double support pose
        traj.get("trunk_pos_x").addPoint(0.0, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(0.0, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(0.0, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(0.0, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(0.0, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(0.0, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(0.0, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(0.0, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(0.0, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);

        //Pre Kick middle
        traj.get("trunk_pos_x").addPoint(middle1Time, trunkPosAtMiddle1.x(), trunkPosVelAtMiddle1.x(), trunkPosAccAtMiddle1.x());
        traj.get("trunk_pos_y").addPoint(middle1Time, trunkPosAtMiddle1.y(), trunkPosVelAtMiddle1.y(), trunkPosAccAtMiddle1.y());
        traj.get("trunk_pos_z").addPoint(middle1Time, trunkPosAtMiddle1.z(), trunkPosVelAtMiddle1.z(), trunkPosAccAtMiddle1.z());
        traj.get("trunk_axis_x").addPoint(middle1Time, trunkAngleAtMiddle1.x(), trunkAngleVelAtMiddle1.x(), trunkAngleAccAtMiddle1.x());
        traj.get("trunk_axis_y").addPoint(middle1Time, trunkAngleAtMiddle1.y(), trunkAngleVelAtMiddle1.y(), trunkAngleAccAtMiddle1.y());
        traj.get("trunk_axis_z").addPoint(middle1Time, trunkAngleAtMiddle1.z(), trunkAngleVelAtMiddle1.z(), trunkAngleAccAtMiddle1.z());
        traj.get("foot_pos_x").addPoint(middle1Time, footPosAtMiddle1.x(), footPosVelAtMiddle1.x(), footPosAccAtMiddle1.x());
        traj.get("foot_pos_y").addPoint(middle1Time, footPosAtMiddle1.y(), footPosVelAtMiddle1.y(), footPosAccAtMiddle1.y());
        traj.get("foot_pos_z").addPoint(middle1Time, footPosAtMiddle1.z(), footPosVelAtMiddle1.z(), footPosAccAtMiddle1.z());
        traj.get("foot_axis_x").addPoint(middle1Time, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(middle1Time, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(middle1Time, 0.0, 0.0);
        
        //Kick contact
        traj.get("trunk_pos_x").addPoint(contactTime, trunkPosAtContact.x(), trunkPosVelAtContact.x(), trunkPosAccAtContact.x());
        traj.get("trunk_pos_y").addPoint(contactTime, trunkPosAtContact.y(), trunkPosVelAtContact.y(), trunkPosAccAtContact.y());
        traj.get("trunk_pos_z").addPoint(contactTime, trunkPosAtContact.z(), trunkPosVelAtContact.z(), trunkPosAccAtContact.z());
        traj.get("trunk_axis_x").addPoint(contactTime, trunkAngleAtContact.x(), trunkAngleVelAtContact.x(), trunkAngleAccAtContact.x());
        traj.get("trunk_axis_y").addPoint(contactTime, trunkAngleAtContact.y(), trunkAngleVelAtContact.y(), trunkAngleAccAtContact.y());
        traj.get("trunk_axis_z").addPoint(contactTime, trunkAngleAtContact.z(), trunkAngleVelAtContact.z(), trunkAngleAccAtContact.z());
        traj.get("foot_pos_x").addPoint(contactTime, footPosAtContact.x(), footPosVelAtContact.x(), footPosAccAtContact.x());
        traj.get("foot_pos_y").addPoint(contactTime, footPosAtContact.y(), footPosVelAtContact.y(), footPosAccAtContact.y());
        traj.get("foot_pos_z").addPoint(contactTime, footPosAtContact.z(), footPosVelAtContact.z(), footPosAccAtContact.z());
        traj.get("foot_axis_x").addPoint(contactTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(contactTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(contactTime, 0.0, 0.0);
        
        //Post Kick middle
        traj.get("trunk_pos_x").addPoint(middle2Time, trunkPosAtMiddle2.x(), trunkPosVelAtMiddle2.x(), trunkPosAccAtMiddle2.x());
        traj.get("trunk_pos_y").addPoint(middle2Time, trunkPosAtMiddle2.y(), trunkPosVelAtMiddle2.y(), trunkPosAccAtMiddle2.y());
        traj.get("trunk_pos_z").addPoint(middle2Time, trunkPosAtMiddle2.z(), trunkPosVelAtMiddle2.z(), trunkPosAccAtMiddle2.z());
        traj.get("trunk_axis_x").addPoint(middle2Time, trunkAngleAtMiddle2.x(), trunkAngleVelAtMiddle2.x(), trunkAngleAccAtMiddle2.x());
        traj.get("trunk_axis_y").addPoint(middle2Time, trunkAngleAtMiddle2.y(), trunkAngleVelAtMiddle2.y(), trunkAngleAccAtMiddle2.y());
        traj.get("trunk_axis_z").addPoint(middle2Time, trunkAngleAtMiddle2.z(), trunkAngleVelAtMiddle2.z(), trunkAngleAccAtMiddle2.z());
        traj.get("foot_pos_x").addPoint(middle2Time, footPosAtMiddle2.x(), footPosVelAtMiddle2.x(), footPosAccAtMiddle2.x());
        traj.get("foot_pos_y").addPoint(middle2Time, footPosAtMiddle2.y(), footPosVelAtMiddle2.y(), footPosAccAtMiddle2.y());
        traj.get("foot_pos_z").addPoint(middle2Time, footPosAtMiddle2.z(), footPosVelAtMiddle2.z(), footPosAccAtMiddle2.z());
        traj.get("foot_axis_x").addPoint(middle2Time, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(middle2Time, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(middle2Time, 0.0, 0.0);
        
        //Ending single support pose
        traj.get("trunk_pos_x").addPoint(endTrajectoryTime, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(endTrajectoryTime, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(endTrajectoryTime, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(endTrajectoryTime, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(endTrajectoryTime, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(endTrajectoryTime, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(endTrajectoryTime, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(endTrajectoryTime, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(endTrajectoryTime, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

        return traj;
    });

    //Set parameters bound function
    generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
        //Check support ratio bound
        if (params(0) <= 0.1) {
            return  1000.0 - 1000.0*params(0);
        }
        if (params(0) >= 0.9) {
            return  1000.0 + 1000.0*(params(0) - 1.0);
        }
        if (params(49) <= 0.1) {
            return  1000.0 - 1000.0*params(49);
        }
        if (params(49) >= 0.9) {
            return  1000.0 + 1000.0*(params(49) - 1.0);
        }
        if (params(50) <= 0.1) {
            return  1000.0 - 1000.0*params(50);
        }
        if (params(50) >= 0.9) {
            return  1000.0 + 1000.0*(params(50) - 1.0);
        }
        if (params(49) + 0.05 > params(0)) {
            return 1000.0 - 1000.0*(params(0)-params(49)-0.05);
        }
        if (params(0) + 0.05 > params(50)) {
            return 1000.0 - 1000.0*(params(50)-params(0)-0.05);
        }
        return 0.0;
    });
    //Set default Cartesian state and Joint DOF bounds
    generator.setCheckStateFunc(Leph::DefaultCheckState);
    generator.setCheckDOFFunc(Leph::DefaultCheckDOF);
    //Set trajectory scoring function
    generator.setScoreFunc([](
        double t,
        Leph::HumanoidFixedModel& model,
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        std::vector<double>& data) -> double {
        (void)t;
        (void)data;
        double cost = 0.0;
        //ZMP
        Eigen::Vector3d zmp = model.zeroMomentPoint("origin", dq, ddq);
        cost += 0.01*torques.norm();
        zmp.z() = 0.0;
        cost += zmp.norm();
        return cost;
    });
    generator.setEndScoreFunc([](
        const Eigen::VectorXd& params,
        const Leph::Trajectories& traj, 
        std::vector<double>& data) -> double {
        (void)params;
        (void)traj;
        (void)data;
        return 0.0;
    });
    
    //Display initial trajectory
    TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

    //Target filename
    std::string filename = "/tmp/trajKick_" + Leph::currentDate() + ".splines";
    //Run the CMA-ES optimization
    generator.runOptimization(2000, 5, filename, 20, -1.0);

    //Display found trajectory
    TrajectoriesDisplay(generator.bestTrajectories());
}

void generateStaticSingleSupport()
{
    //Initialize the generator
    Leph::TrajectoryGeneration generator(Leph::SigmabanModel);
    
    //Initial trajectory parameters
    generator.setInitialParameters([]() -> Eigen::VectorXd {
        Eigen::VectorXd params(9);
        params(0) = initTrunkPos().x();
        params(1) = initTrunkPos().y();
        params(2) = initTrunkPos().z();
        params(3) = initTrunkAxis().x();
        params(4) = initTrunkAxis().y();
        params(5) = initTrunkAxis().z();
        params(6) = initFootPos().x();
        params(7) = initFootPos().y();
        params(8) = initFootPos().z();
        
        return params;
    }());

    //Set Trajectory generation function
    generator.setTrajectoryGenerationFunc([](const Eigen::VectorXd& params) -> Leph::Trajectories {
        double endTrajectoryTime = 0.1;
        Eigen::Vector3d trunkPos = params.segment<3>(0);
        Eigen::Vector3d trunkAxis = params.segment<3>(3);
        Eigen::Vector3d footPos = params.segment<3>(6);
        
        //Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        //Support phase (single support for kick)
        traj.get("is_double_support").addPoint(0.0, 0.0, 0.0);
        traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);
        //Support foot 
        traj.get("is_left_support_foot").addPoint(0.0, 1.0);
        traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0);

        traj.get("trunk_pos_x").addPoint(0.0, trunkPos.x(), 0.0);
        traj.get("trunk_pos_y").addPoint(0.0, trunkPos.y(), 0.0);
        traj.get("trunk_pos_z").addPoint(0.0, trunkPos.z(), 0.0);
        traj.get("trunk_axis_x").addPoint(0.0, trunkAxis.x(), 0.0);
        traj.get("trunk_axis_y").addPoint(0.0, trunkAxis.y(), 0.0);
        traj.get("trunk_axis_z").addPoint(0.0, trunkAxis.z(), 0.0);
        traj.get("foot_pos_x").addPoint(0.0, footPos.x(), 0.0);
        traj.get("foot_pos_y").addPoint(0.0, footPos.y(), 0.0);
        traj.get("foot_pos_z").addPoint(0.0, footPos.z(), 0.0);
        traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);
        
        traj.get("trunk_pos_x").addPoint(endTrajectoryTime, trunkPos.x(), 0.0);
        traj.get("trunk_pos_y").addPoint(endTrajectoryTime, trunkPos.y(), 0.0);
        traj.get("trunk_pos_z").addPoint(endTrajectoryTime, trunkPos.z(), 0.0);
        traj.get("trunk_axis_x").addPoint(endTrajectoryTime, trunkAxis.x(), 0.0);
        traj.get("trunk_axis_y").addPoint(endTrajectoryTime, trunkAxis.y(), 0.0);
        traj.get("trunk_axis_z").addPoint(endTrajectoryTime, trunkAxis.z(), 0.0);
        traj.get("foot_pos_x").addPoint(endTrajectoryTime, footPos.x(), 0.0);
        traj.get("foot_pos_y").addPoint(endTrajectoryTime, footPos.y(), 0.0);
        traj.get("foot_pos_z").addPoint(endTrajectoryTime, footPos.z(), 0.0);
        traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

        return traj;
    });

    //Set parameters bound function
    generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
        (void)params;
        return 0.0;
    });
    //Set default Cartesian state and Joint DOF bounds
    generator.setCheckStateFunc(Leph::DefaultCheckState);
    generator.setCheckDOFFunc(Leph::DefaultCheckDOF);
    //Set trajectory scoring function
    generator.setScoreFunc([](
        double t,
        Leph::HumanoidFixedModel& model,
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        std::vector<double>& data) -> double {
        
        (void)t;
        (void)model;
        (void)dq;
        (void)ddq;
        (void)data;
        double cost = 0.0;
        cost += 0.01*torques.norm();
        return cost;
    });
    generator.setEndScoreFunc([](
        const Eigen::VectorXd& params,
        const Leph::Trajectories& traj, 
        std::vector<double>& data) -> double {
        (void)params;
        (void)data;
        (void)traj;
        return 0.0;
    });
    
    //Display initial trajectory
    TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

    //Target filename
    std::string filename = "/tmp/trajStatic_" + Leph::currentDate() + ".splines";
    //Run the CMA-ES optimization
    generator.runOptimization(1000, 5, filename, 20, -1.0);

    //Display found trajectory
    TrajectoriesDisplay(generator.bestTrajectories());
}

void generateRecovery()
{
    //Initialize the generator
    Leph::TrajectoryGeneration generator(Leph::SigmabanModel);
    
    //Initial trajectory parameters
    generator.setInitialParameters([]() -> Eigen::VectorXd {
        Eigen::VectorXd params(48);
        //Time ratio
        params(0) = 0.2;
        params(1) = 0.5;
        params(2) = 0.8;

        //Pos time 1
        params(3) = targetTrunkPos().x() - 0.03;
        params(4) = targetTrunkPos().y();
        params(9) = targetTrunkAxis().x();
        params(10) = targetTrunkAxis().y();
        params(11) = targetTrunkAxis().z();
        //Vel/Acc time 1
        params.segment<2>(5).setZero();
        params.segment<2>(7).setZero();
        params.segment<3>(12).setZero();
        params.segment<3>(15).setZero();
        
        //Pos time 2
        params(18) = targetTrunkPos().x() - 0.03;
        params(19) = targetTrunkPos().y();
        params(24) = targetTrunkAxis().x();
        params(25) = targetTrunkAxis().y();
        params(26) = targetTrunkAxis().z();
        //Vel/Acc time 2
        params.segment<2>(20).setZero();
        params.segment<2>(22).setZero();
        params.segment<3>(27).setZero();
        params.segment<3>(30).setZero();
        
        //Pos time 3
        params(33) = targetTrunkPos().x() - 0.03;
        params(34) = targetTrunkPos().y();
        params(39) = targetTrunkAxis().x();
        params(40) = targetTrunkAxis().y();
        params(41) = targetTrunkAxis().z();
        //Vel/Acc time 3
        params.segment<2>(35).setZero();
        params.segment<2>(37).setZero();
        params.segment<3>(42).setZero();
        params.segment<3>(45).setZero();
        
        return params;
    }());

    double endTrajectoryTime = 2.0;
    //Set Trajectory generation function
    generator.setTrajectoryGenerationFunc([&endTrajectoryTime](const Eigen::VectorXd& params) -> Leph::Trajectories {
        double time1 = params(0) * endTrajectoryTime;
        double time2 = params(1) * endTrajectoryTime;
        double time3 = params(2) * endTrajectoryTime;

        Eigen::Vector3d trunkPosAt1(params(3), params(4), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAt1(params(5), params(6), 0.0);
        Eigen::Vector3d trunkPosAccAt1(params(7), params(8), 0.0);
        Eigen::Vector3d trunkAxisAt1 = params.segment<3>(9);
        Eigen::Vector3d trunkAxisVelAt1 = params.segment<3>(12);
        Eigen::Vector3d trunkAxisAccAt1 = params.segment<3>(15);
        
        Eigen::Vector3d trunkPosAt2(params(18), params(19), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAt2(params(20), params(21), 0.0);
        Eigen::Vector3d trunkPosAccAt2(params(22), params(23), 0.0);
        Eigen::Vector3d trunkAxisAt2 = params.segment<3>(24);
        Eigen::Vector3d trunkAxisVelAt2 = params.segment<3>(27);
        Eigen::Vector3d trunkAxisAccAt2 = params.segment<3>(30);
        
        Eigen::Vector3d trunkPosAt3(params(33), params(34), targetTrunkPos().z());
        Eigen::Vector3d trunkPosVelAt3(params(35), params(36), 0.0);
        Eigen::Vector3d trunkPosAccAt3(params(37), params(38), 0.0);
        Eigen::Vector3d trunkAxisAt3 = params.segment<3>(39);
        Eigen::Vector3d trunkAxisVelAt3 = params.segment<3>(42);
        Eigen::Vector3d trunkAxisAccAt3 = params.segment<3>(45);
        
        //Initialize state splines
        Leph::Trajectories traj = Leph::TrajectoriesInit();

        //Support phase (single support for kick)
        traj.get("is_double_support").addPoint(0.0, 0.0, 0.0);
        traj.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);
        //Support foot 
        traj.get("is_left_support_foot").addPoint(0.0, 1.0);
        traj.get("is_left_support_foot").addPoint(endTrajectoryTime, 1.0);

        //Starting position
        traj.get("trunk_pos_x").addPoint(0.0, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(0.0, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(0.0, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(0.0, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(0.0, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(0.0, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(0.0, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(0.0, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(0.0, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(0.0, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(0.0, 0.0, 0.0);
        
        //Time 1
        traj.get("trunk_pos_x").addPoint(time1, trunkPosAt1.x(), trunkPosVelAt1.x(), trunkPosAccAt1.x());
        traj.get("trunk_pos_y").addPoint(time1, trunkPosAt1.y(), trunkPosVelAt1.y(), trunkPosAccAt1.y());
        traj.get("trunk_pos_z").addPoint(time1, trunkPosAt1.z(), trunkPosVelAt1.z(), trunkPosAccAt1.z());
        traj.get("trunk_axis_x").addPoint(time1, trunkAxisAt1.x(), trunkAxisVelAt1.x(), trunkAxisAccAt1.x());
        traj.get("trunk_axis_y").addPoint(time1, trunkAxisAt1.y(), trunkAxisVelAt1.y(), trunkAxisAccAt1.y());
        traj.get("trunk_axis_z").addPoint(time1, trunkAxisAt1.z(), trunkAxisVelAt1.z(), trunkAxisAccAt1.z());
        //Time 2
        traj.get("trunk_pos_x").addPoint(time2, trunkPosAt2.x(), trunkPosVelAt2.x(), trunkPosAccAt2.x());
        traj.get("trunk_pos_y").addPoint(time2, trunkPosAt2.y(), trunkPosVelAt2.y(), trunkPosAccAt2.y());
        traj.get("trunk_pos_z").addPoint(time2, trunkPosAt2.z(), trunkPosVelAt2.z(), trunkPosAccAt2.z());
        traj.get("trunk_axis_x").addPoint(time2, trunkAxisAt2.x(), trunkAxisVelAt2.x(), trunkAxisAccAt2.x());
        traj.get("trunk_axis_y").addPoint(time2, trunkAxisAt2.y(), trunkAxisVelAt2.y(), trunkAxisAccAt2.y());
        traj.get("trunk_axis_z").addPoint(time2, trunkAxisAt2.z(), trunkAxisVelAt2.z(), trunkAxisAccAt2.z());
        //Time 3
        traj.get("trunk_pos_x").addPoint(time3, trunkPosAt3.x(), trunkPosVelAt3.x(), trunkPosAccAt3.x());
        traj.get("trunk_pos_y").addPoint(time3, trunkPosAt3.y(), trunkPosVelAt3.y(), trunkPosAccAt3.y());
        traj.get("trunk_pos_z").addPoint(time3, trunkPosAt3.z(), trunkPosVelAt3.z(), trunkPosAccAt3.z());
        traj.get("trunk_axis_x").addPoint(time3, trunkAxisAt3.x(), trunkAxisVelAt3.x(), trunkAxisAccAt3.x());
        traj.get("trunk_axis_y").addPoint(time3, trunkAxisAt3.y(), trunkAxisVelAt3.y(), trunkAxisAccAt3.y());
        traj.get("trunk_axis_z").addPoint(time3, trunkAxisAt3.z(), trunkAxisVelAt3.z(), trunkAxisAccAt3.z());
        
        //Final position
        traj.get("trunk_pos_x").addPoint(endTrajectoryTime, targetTrunkPos().x(), 0.0);
        traj.get("trunk_pos_y").addPoint(endTrajectoryTime, targetTrunkPos().y(), 0.0);
        traj.get("trunk_pos_z").addPoint(endTrajectoryTime, targetTrunkPos().z(), 0.0);
        traj.get("trunk_axis_x").addPoint(endTrajectoryTime, targetTrunkAxis().x(), 0.0);
        traj.get("trunk_axis_y").addPoint(endTrajectoryTime, targetTrunkAxis().y(), 0.0);
        traj.get("trunk_axis_z").addPoint(endTrajectoryTime, targetTrunkAxis().z(), 0.0);
        traj.get("foot_pos_x").addPoint(endTrajectoryTime, targetFootPos().x(), 0.0);
        traj.get("foot_pos_y").addPoint(endTrajectoryTime, targetFootPos().y(), 0.0);
        traj.get("foot_pos_z").addPoint(endTrajectoryTime, targetFootPos().z(), 0.0);
        traj.get("foot_axis_x").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_y").addPoint(endTrajectoryTime, 0.0, 0.0);
        traj.get("foot_axis_z").addPoint(endTrajectoryTime, 0.0, 0.0);

        return traj;
    });

    //Set parameters bound function
    generator.setCheckParametersFunc([](const Eigen::VectorXd& params) -> double {
        if (params(0) <= 0.1) {
            return 1000.0 - 1000.0*(params(0) - 0.1);
        }
        if (params(0) >= 0.9) {
            return 1000.0 + 1000.0*(params(0) - 0.9);
        }
        if (params(1) <= 0.1) {
            return 1000.0 - 1000.0*(params(1) - 0.1);
        }
        if (params(1) >= 0.9) {
            return 1000.0 + 1000.0*(params(1) - 0.9);
        }
        if (params(2) <= 0.1) {
            return 1000.0 - 1000.0*(params(2) - 0.1);
        }
        if (params(2) >= 0.9) {
            return 1000.0 + 1000.0*(params(2) - 0.9);
        }
        if (params(0) + 0.1 > params(1)) {
            return 1000.0 - 1000.0*(params(1)-params(0)-0.1);
        }
        if (params(1) + 0.1 > params(2)) {
            return 1000.0 - 1000.0*(params(2)-params(1)-0.1);
        }
        return 0.0;
    });
    //Set default Cartesian state and Joint DOF bounds
    generator.setCheckStateFunc(Leph::DefaultCheckState);
    generator.setCheckDOFFunc(Leph::DefaultCheckDOF);

    Leph::SmoothSpline zmpSpline;
    zmpSpline.addPoint(0.0, 0.0);
    zmpSpline.addPoint(0.5, -0.02);
    zmpSpline.addPoint(endTrajectoryTime, 0.0);
    zmpSpline.plot().plot("t", "pos:*").render();

    //Set trajectory scoring function
    generator.setScoreFunc([&zmpSpline](
        double t,
        Leph::HumanoidFixedModel& model,
        const Eigen::VectorXd& torques,
        const Eigen::VectorXd& dq,
        const Eigen::VectorXd& ddq,
        std::vector<double>& data) -> double {
        
        (void)model;
        (void)data;
        double cost = 0.0;
        cost += 0.001*torques.norm();
        Eigen::Vector3d zmp = model.zeroMomentPoint("origin", dq, ddq);
        cost += fabs(zmp.y());
        cost += fabs(zmpSpline.pos(t) - zmp.x());
        /*
        if (zmp.x() > 0.0) {
            cost += 5.0*zmp.x();
        }
        */

        return cost;
    });
    generator.setEndScoreFunc([](
        const Eigen::VectorXd& params,
        const Leph::Trajectories& traj, 
        std::vector<double>& data) -> double {
        (void)params;
        (void)traj;
        (void)data;
        return 0.0;
    });
    
    //Display initial trajectory
    TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));

    //Target filename
    std::string filename = "/tmp/trajRecovery_" + Leph::currentDate() + ".splines";
    //Run the CMA-ES optimization
    generator.runOptimization(2000, 5, filename, 20, -1.0);

    //Display found trajectory
    TrajectoriesDisplay(generator.bestTrajectories());
}

int main(int argc, char** argv)
{
    //Check command line
    if (argc < 2) {
        std::cout << "Usage: ./app [kick|leglift|static|recovery]" << std::endl;
        return 1;
    }
    std::string mode = std::string(argv[1]);

    if (mode == "kick") {
        generateKick();
    } else if (mode == "leglift") {
        generateLegLift();
    } else if (mode == "static") {
        generateStaticSingleSupport();
    } else if (mode == "recovery") {
        generateRecovery();
    } else {
        std::cout << "Bad mode: " + mode << std::endl;
    }

    return 0;
}

