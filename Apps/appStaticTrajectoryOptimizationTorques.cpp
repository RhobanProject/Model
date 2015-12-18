#include <iostream>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Spline/CubicSpline.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"
#include "Utils/AxisAngle.h"
#include "Spline/SplineContainer.hpp"

/**
 * Global lenght of trajectory
 */
double TrajectoryLength = 3.0;

/**
 * Return fitness cost if state 
 * bounds are not valid
 */
static double boundState(
    Eigen::Vector3d& trunkPos,
    Eigen::Vector3d& trunkAxisAngles,
    Eigen::Vector3d& flyingFootPos)
{
    double cost = 0.0;
    if (trunkPos.z() < 0.0) {
        cost += 100.0 - 1000.0*trunkPos.z();
    }
    if (trunkAxisAngles.norm() > M_PI/2.0) {
        cost += 100.0 + 1000.0*(trunkAxisAngles.norm() - M_PI/2.0);
    }
    if (flyingFootPos.y() > -2.0*0.039995) {
        cost += 100.0 + 1000.0*(flyingFootPos.y() + 2.0*0.039995);
    }
    if (flyingFootPos.z() < 0.0) {
        cost += 100.0 - 1000.0*flyingFootPos.z();
    }

    return cost;
}

/**
 * Return fitness cost for not valid
 * DOF range
 */
static double boundDOF(const Leph::Model& model)
{
    double cost = 0.0;

    if (fabs(model.getDOF("left_hip_yaw")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("left_hip_yaw")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("left_hip_roll")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("left_hip_roll")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("left_hip_pitch")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("left_hip_pitch")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("left_knee")) > 3.0*M_PI/2.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("left_knee")) - 3.0*M_PI/2.0);
    }
    if (fabs(model.getDOF("left_ankle_pitch")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("left_ankle_pitch")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("left_ankle_roll")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("left_ankle_roll")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_hip_yaw")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("right_hip_yaw")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_hip_roll")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("right_hip_roll")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_hip_pitch")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("right_hip_pitch")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_knee")) > 3.0*M_PI/2.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("right_knee")) - 3.0*M_PI/2.0);
    }
    if (fabs(model.getDOF("right_ankle_pitch")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("right_ankle_pitch")) - M_PI/3.0);
    }
    if (fabs(model.getDOF("right_ankle_roll")) > M_PI/3.0) {
        cost += 100.0 + 1000.0*(fabs(model.getDOF("right_ankle_roll")) - M_PI/3.0);
    }

    return cost;
}

/**
 * Ending precompute optimal state
 */
static Eigen::Vector3d targetTrunkPos()
{
    return Eigen::Vector3d(-0.00421393418876207, -0.0126689516724556, 0.29126642326467);
}
static Eigen::Vector3d targetTrunkAxisAngles()
{
    return Eigen::Vector3d(-0.653138039395134, 0.056656261598521, 0.0496181962098105);
}
static Eigen::Vector3d targetFlyingFootPos()
{
    return Eigen::Vector3d(0.0253025113778088, -0.0799900000041802, 0.0570644312158811);
}

/**
 * Return initial state
 */
static Eigen::Vector3d initTrunkPos()
{
    return Eigen::Vector3d(0.0, -0.05, 0.20);
}
static Eigen::Vector3d initTrunkAxisAngles()
{
    return Eigen::Vector3d(0.0, 0.0, 0.0);
}
static Eigen::Vector3d initFlyingFootPos()
{
    return Eigen::Vector3d(0.0, -0.1, 0.0);
}

/**
 * Precompute optimal torque minimal sigle support pose
 */
static void computeOptimalPose()
{
    //Sigmaban fixed model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    //Initial position
    Eigen::VectorXd initParams(9);
    initParams.segment(0, 3) = initTrunkPos();
    initParams.segment(3, 3) = initTrunkAxisAngles();
    initParams.segment(6, 3) = initFlyingFootPos();
    //Try to find minimum torques configuration with
    //CMA-ES optimization
    libcmaes::FitFuncEigen fitness = [&model]
        (const Eigen::VectorXd& params) 
    {
        double cost = 0.0;
        //Parameters extraction
        Eigen::Vector3d trunkPos = params.segment<3>(0);
        Eigen::Vector3d trunkAxisAngles = params.segment<3>(3);
        Eigen::Vector3d flyingFootPos = params.segment<3>(6);
        //Check AxisAngle norm
        if (trunkAxisAngles.norm() > M_PI/2.0) {
            return 1000.0 + trunkAxisAngles.norm() - M_PI/2.0;
        }
        //Bound parameters
        cost += boundState(trunkPos, trunkAxisAngles, flyingFootPos);
        //Set model state
        bool isSuccess = model.trunkFootIK(
            Leph::HumanoidFixedModel::LeftSupportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAxisAngles),
            flyingFootPos);
        if (!isSuccess) {
            //Penalize IK error
            cost += 1000.0;
        } else {
            //Penalize DOF range
            cost += boundDOF(model.get());
            //Compute single support static torques
            Eigen::VectorXd tau = model.get().inverseDynamics();
            tau(model.get().getDOFIndex("base_x")) = 0.0;
            tau(model.get().getDOFIndex("base_y")) = 0.0;
            tau(model.get().getDOFIndex("base_z")) = 0.0;
            cost += tau.norm();
        }

        return cost;
    };
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(initParams, -1.0, 100);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(false);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(true);
    cmaparams.set_max_iter(10000);
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams);
    Eigen::VectorXd bestParams = 
        cmasols.get_best_seen_candidate().get_x_dvec();
    double score = 
        cmasols.get_best_seen_candidate().get_fvalue();
    //Display resulting pose
    std::cout << "Score: " << score << std::endl;
    std::cout << "TrunkPos: " << bestParams.segment<3>(0).transpose() << std::endl;
    std::cout << "TrunkAxisAngles: " << bestParams.segment<3>(3).transpose() << std::endl;
    std::cout << "FootPos: " << bestParams.segment<3>(6).transpose() << std::endl;
    //Viewer
    Leph::ModelViewer viewer(1200, 900);
    model.trunkFootIK(
        Leph::HumanoidFixedModel::LeftSupportFoot,
        bestParams.segment<3>(0),
        Leph::AxisToMatrix(bestParams.segment<3>(3)),
        bestParams.segment<3>(6));
    //Display best found pose
    while (viewer.update()) {
        Leph::ModelDraw(model.get(), viewer);
    }
}

/**
 * Return initial trajectory parameters
 */
static Eigen::VectorXd initParameters()
{
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
    params(7) = 0.5*initTrunkAxisAngles().x() + 0.5*targetTrunkAxisAngles().x();
    params(8) = 0.5*initTrunkAxisAngles().y() + 0.5*targetTrunkAxisAngles().y();
    params(9) = 0.5*initTrunkAxisAngles().z() + 0.5*targetTrunkAxisAngles().z();
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
    params(19) = 0.75*initTrunkAxisAngles().x() + 0.25*targetTrunkAxisAngles().x();
    params(20) = 0.75*initTrunkAxisAngles().y() + 0.25*targetTrunkAxisAngles().y();
    params(21) = 0.75*initTrunkAxisAngles().z() + 0.25*targetTrunkAxisAngles().z();
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
    params(31) = 0.25*initTrunkAxisAngles().x() + 0.75*targetTrunkAxisAngles().x();
    params(32) = 0.25*initTrunkAxisAngles().y() + 0.75*targetTrunkAxisAngles().y();
    params(33) = 0.25*initTrunkAxisAngles().z() + 0.75*targetTrunkAxisAngles().z();
    //Middle 2 trunk vel
    params(34) = 0.0;
    params(35) = 0.0;
    params(36) = 0.0;
    //Middle 2 foot pos
    params(37) = 0.5*initFlyingFootPos().x() + 0.5*targetFlyingFootPos().x();
    params(38) = 0.5*initFlyingFootPos().y() + 0.5*targetFlyingFootPos().y();
    params(39) = 0.5*initFlyingFootPos().z() + 0.5*targetFlyingFootPos().z();
    //Middle 2 foot vel
    params(40) = 0.0;
    params(41) = 0.0;
    params(42) = 0.0;

    return params;
}

/**
 * Build and return foot position, trunk position and orientation
 * state splines trajectory with given parameters Vector
 */
static Leph::SplineContainer<Leph::CubicSpline> generateTrajectory(const Eigen::VectorXd& params)
{
    //Extract parameters
    double endTrajectoryTime = TrajectoryLength;
    double ratioSwapSupport = params(0);
    double swapSupportTime = ratioSwapSupport*endTrajectoryTime;
    Eigen::Vector3d trunkPosAtSwap = params.segment<3>(1);
    Eigen::Vector3d trunkPosVelAtSwap = params.segment<3>(4);
    Eigen::Vector3d trunkAngleAtSwap = params.segment<3>(7);
    Eigen::Vector3d trunkAngleVelAtSwap = params.segment<3>(10);
    Eigen::Vector3d footPosAtSwap = initFlyingFootPos();
    
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
    Leph::SplineContainer<Leph::CubicSpline> container;
    container.add("trunk_pos_x");
    container.add("trunk_pos_y");
    container.add("trunk_pos_z");
    container.add("trunk_axis_x");
    container.add("trunk_axis_y");
    container.add("trunk_axis_z");
    container.add("foot_pos_x");
    container.add("foot_pos_y");
    container.add("foot_pos_z");
    container.add("is_double_support");

    //Support phase
    container.get("is_double_support").addPoint(0.0, 1.0, 0.0);
    container.get("is_double_support").addPoint(swapSupportTime, 1.0, 0.0);
    container.get("is_double_support").addPoint(swapSupportTime, 0.0, 0.0);
    container.get("is_double_support").addPoint(endTrajectoryTime, 0.0, 0.0);

    //Starting double support pose
    container.get("trunk_pos_x").addPoint(0.0, initTrunkPos().x(), 0.0);
    container.get("trunk_pos_y").addPoint(0.0, initTrunkPos().y(), 0.0);
    container.get("trunk_pos_z").addPoint(0.0, initTrunkPos().z(), 0.0);
    container.get("trunk_axis_x").addPoint(0.0, initTrunkAxisAngles().x(), 0.0);
    container.get("trunk_axis_y").addPoint(0.0, initTrunkAxisAngles().y(), 0.0);
    container.get("trunk_axis_z").addPoint(0.0, initTrunkAxisAngles().z(), 0.0);
    container.get("foot_pos_x").addPoint(0.0, initFlyingFootPos().x(), 0.0);
    container.get("foot_pos_y").addPoint(0.0, initFlyingFootPos().y(), 0.0);
    container.get("foot_pos_z").addPoint(0.0, initFlyingFootPos().z(), 0.0);

    //Double support phase middle
    container.get("trunk_pos_x").addPoint(middle1Time, trunkPosAtMiddle1.x(), trunkPosVelAtMiddle1.x());
    container.get("trunk_pos_y").addPoint(middle1Time, trunkPosAtMiddle1.y(), trunkPosVelAtMiddle1.y());
    container.get("trunk_pos_z").addPoint(middle1Time, trunkPosAtMiddle1.z(), trunkPosVelAtMiddle1.z());
    container.get("trunk_axis_x").addPoint(middle1Time, trunkAngleAtMiddle1.x(), trunkAngleVelAtMiddle1.x());
    container.get("trunk_axis_y").addPoint(middle1Time, trunkAngleAtMiddle1.y(), trunkAngleVelAtMiddle1.y());
    container.get("trunk_axis_z").addPoint(middle1Time, trunkAngleAtMiddle1.z(), trunkAngleVelAtMiddle1.z());
    
    //Double to single support phase
    container.get("trunk_pos_x").addPoint(swapSupportTime, trunkPosAtSwap.x(), trunkPosVelAtSwap.x());
    container.get("trunk_pos_y").addPoint(swapSupportTime, trunkPosAtSwap.y(), trunkPosVelAtSwap.y());
    container.get("trunk_pos_z").addPoint(swapSupportTime, trunkPosAtSwap.z(), trunkPosVelAtSwap.z());
    container.get("trunk_axis_x").addPoint(swapSupportTime, trunkAngleAtSwap.x(), trunkAngleVelAtSwap.x());
    container.get("trunk_axis_y").addPoint(swapSupportTime, trunkAngleAtSwap.y(), trunkAngleVelAtSwap.y());
    container.get("trunk_axis_z").addPoint(swapSupportTime, trunkAngleAtSwap.z(), trunkAngleVelAtSwap.z());
    container.get("foot_pos_x").addPoint(swapSupportTime, footPosAtSwap.x(), 0.0);
    container.get("foot_pos_y").addPoint(swapSupportTime, footPosAtSwap.y(), 0.0);
    container.get("foot_pos_z").addPoint(swapSupportTime, footPosAtSwap.z(), 0.0);
    
    //Double support phase middle
    container.get("trunk_pos_x").addPoint(middle2Time, trunkPosAtMiddle2.x(), trunkPosVelAtMiddle2.x());
    container.get("trunk_pos_y").addPoint(middle2Time, trunkPosAtMiddle2.y(), trunkPosVelAtMiddle2.y());
    container.get("trunk_pos_z").addPoint(middle2Time, trunkPosAtMiddle2.z(), trunkPosVelAtMiddle2.z());
    container.get("trunk_axis_x").addPoint(middle2Time, trunkAngleAtMiddle2.x(), trunkAngleVelAtMiddle2.x());
    container.get("trunk_axis_y").addPoint(middle2Time, trunkAngleAtMiddle2.y(), trunkAngleVelAtMiddle2.y());
    container.get("trunk_axis_z").addPoint(middle2Time, trunkAngleAtMiddle2.z(), trunkAngleVelAtMiddle2.z());
    container.get("foot_pos_x").addPoint(middle2Time, footPosAtMiddle2.x(), footPosVelAtMiddle2.x());
    container.get("foot_pos_y").addPoint(middle2Time, footPosAtMiddle2.y(), footPosVelAtMiddle2.y());
    container.get("foot_pos_z").addPoint(middle2Time, footPosAtMiddle2.z(), footPosVelAtMiddle2.z());
    
    //Ending single support pose
    container.get("trunk_pos_x").addPoint(endTrajectoryTime, targetTrunkPos().x(), 0.0);
    container.get("trunk_pos_y").addPoint(endTrajectoryTime, targetTrunkPos().y(), 0.0);
    container.get("trunk_pos_z").addPoint(endTrajectoryTime, targetTrunkPos().z(), 0.0);
    container.get("trunk_axis_x").addPoint(endTrajectoryTime, targetTrunkAxisAngles().x(), 0.0);
    container.get("trunk_axis_y").addPoint(endTrajectoryTime, targetTrunkAxisAngles().y(), 0.0);
    container.get("trunk_axis_z").addPoint(endTrajectoryTime, targetTrunkAxisAngles().z(), 0.0);
    container.get("foot_pos_x").addPoint(endTrajectoryTime, targetFlyingFootPos().x(), 0.0);
    container.get("foot_pos_y").addPoint(endTrajectoryTime, targetFlyingFootPos().y(), 0.0);
    container.get("foot_pos_z").addPoint(endTrajectoryTime, targetFlyingFootPos().z(), 0.0);

    return container;
}

/**
 * Return the fitness score of the trajectory
 * computed from given parameters
 */
static double scoreTrajectory(const Leph::SplineContainer<Leph::CubicSpline>& container, bool noBound = false)
{
    double cost = 0.0;
    //Sigmaban fixed model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    //Iterate over the trajectory
    for (double t=0;t<container.max();t+=0.01) {
        //Compute current positions
        Eigen::Vector3d trunkPos = 
            Eigen::Vector3d(
                container.get("trunk_pos_x").pos(t), 
                container.get("trunk_pos_y").pos(t), 
                container.get("trunk_pos_z").pos(t));
        Eigen::Vector3d trunkAxisAngles =
            Eigen::Vector3d(
                container.get("trunk_axis_x").pos(t), 
                container.get("trunk_axis_y").pos(t), 
                container.get("trunk_axis_z").pos(t));
        Eigen::Vector3d footPos = 
            Eigen::Vector3d(
                container.get("foot_pos_x").pos(t), 
                container.get("foot_pos_y").pos(t), 
                container.get("foot_pos_z").pos(t));
        bool isDoubleSupport = (bool)container.get("is_double_support").pos(t);
        //Compute current velocities
        Eigen::Vector3d trunkPosVel = 
            Eigen::Vector3d(
                container.get("trunk_pos_x").vel(t), 
                container.get("trunk_pos_y").vel(t), 
                container.get("trunk_pos_z").vel(t));
        Eigen::Vector3d trunkAxisAnglesVel =
            Eigen::Vector3d(
                container.get("trunk_axis_x").vel(t), 
                container.get("trunk_axis_y").vel(t), 
                container.get("trunk_axis_z").vel(t));
        Eigen::Vector3d footPosVel = 
            Eigen::Vector3d(
                container.get("foot_pos_x").vel(t), 
                container.get("foot_pos_y").vel(t), 
                container.get("foot_pos_z").vel(t));
        //Compute current accelerations
        Eigen::Vector3d trunkPosAcc = 
            Eigen::Vector3d(
                container.get("trunk_pos_x").acc(t), 
                container.get("trunk_pos_y").acc(t), 
                container.get("trunk_pos_z").acc(t));
        Eigen::Vector3d trunkAxisAnglesAcc =
            Eigen::Vector3d(
                container.get("trunk_axis_x").acc(t), 
                container.get("trunk_axis_y").acc(t), 
                container.get("trunk_axis_z").acc(t));
        Eigen::Vector3d footPosAcc = 
            Eigen::Vector3d(
                container.get("foot_pos_x").acc(t), 
                container.get("foot_pos_y").acc(t), 
                container.get("foot_pos_z").acc(t));
        //Bound state
        if (!noBound) {
            cost += boundState(trunkPos, trunkAxisAngles, footPos);
        }
        //Assign model state
        bool isSuccess = model.trunkFootIK(
            Leph::HumanoidFixedModel::LeftSupportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAxisAngles),
            footPos);
        //Penalize IK error
        if (!isSuccess) {
            cost += 1000.0;
        }
        //Penalize DOF range
        if (!noBound) {
            cost += boundDOF(model.get());
        }
        //Stop evaluation if cost is too hight
        if (cost > 100.0) {
            break;
        }
        //Compute joints velocities and accelerations
        //Axis differentiation is converted in proper angular
        //velocity and acceleration
        Eigen::VectorXd dq = model.trunkFootIKVel(
            trunkPosVel, 
            Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesVel), 
            footPosVel);
        Eigen::VectorXd ddq = model.trunkFootIKAcc(
            dq,
            trunkPosVel, 
            Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesVel), 
            footPosVel,
            trunkPosAcc, 
            Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesAcc), 
            footPosAcc);
        //Compute static torques
        if (isDoubleSupport) {
            Eigen::VectorXd tau = model.get().inverseDynamicsClosedLoop(
                "right_foot_tip", false, dq, ddq);
            tau(model.get().getDOFIndex("base_x")) = 0.0;
            tau(model.get().getDOFIndex("base_y")) = 0.0;
            tau(model.get().getDOFIndex("base_z")) = 0.0;
            cost += 0.01*tau.norm();
        } else {
            //Compute single support static torques
            Eigen::VectorXd tau = model.get().inverseDynamics(dq, ddq);
            tau(model.get().getDOFIndex("base_x")) = 0.0;
            tau(model.get().getDOFIndex("base_y")) = 0.0;
            tau(model.get().getDOFIndex("base_z")) = 0.0;
            cost += 0.01*tau.norm();
        }
    }

    return cost;
}
static double scoreTrajectory(const Eigen::VectorXd params)
{
    //Check support ratio bound
    if (params(0) <= 0.0) {
        return  1000.0 - 1000.0*params(0);
    }
    if (params(0) >= 1.0) {
        return  1000.0 + 1000.0*(params(0) - 1.0);
    }
    //Compute the trajectory
    Leph::SplineContainer<Leph::CubicSpline> container = 
        generateTrajectory(params);
    //Prevent to short double or single support phase
    if (params(0)*container.max() < 0.5) {
        return 1000.0;
    }
    if (params(0)*container.max() > container.max()-0.5) {
        return 1000.0;
    }
    //Compute the trajectory
    return scoreTrajectory(container);
}

/**
 * Display in viewer the trajectory computed from
 * given parameters
 */
static void displayTrajectory(Leph::ModelViewer& viewer, 
    const Leph::SplineContainer<Leph::CubicSpline>& container)
{
    //Sigmaban fixed model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    //Display trajectory
    Leph::Scheduling scheduling;
    scheduling.setFrequency(50.0);
    double t = 0.0;
    Leph::Plot plot;
    bool isLoop = false;
    while (viewer.update()) {
        //Compute current positions
        Eigen::Vector3d trunkPos = 
            Eigen::Vector3d(
                container.get("trunk_pos_x").pos(t), 
                container.get("trunk_pos_y").pos(t), 
                container.get("trunk_pos_z").pos(t));
        Eigen::Vector3d trunkAxisAngles =
            Eigen::Vector3d(
                container.get("trunk_axis_x").pos(t), 
                container.get("trunk_axis_y").pos(t), 
                container.get("trunk_axis_z").pos(t));
        Eigen::Vector3d footPos = 
            Eigen::Vector3d(
                container.get("foot_pos_x").pos(t), 
                container.get("foot_pos_y").pos(t), 
                container.get("foot_pos_z").pos(t));
        bool isDoubleSupport = (bool)container.get("is_double_support").pos(t);
        //Compute current velocities
        Eigen::Vector3d trunkPosVel = 
            Eigen::Vector3d(
                container.get("trunk_pos_x").vel(t), 
                container.get("trunk_pos_y").vel(t), 
                container.get("trunk_pos_z").vel(t));
        Eigen::Vector3d trunkAxisAnglesVel =
            Eigen::Vector3d(
                container.get("trunk_axis_x").vel(t), 
                container.get("trunk_axis_y").vel(t), 
                container.get("trunk_axis_z").vel(t));
        Eigen::Vector3d footPosVel = 
            Eigen::Vector3d(
                container.get("foot_pos_x").vel(t), 
                container.get("foot_pos_y").vel(t), 
                container.get("foot_pos_z").vel(t));
        //Compute current accelerations
        Eigen::Vector3d trunkPosAcc = 
            Eigen::Vector3d(
                container.get("trunk_pos_x").acc(t), 
                container.get("trunk_pos_y").acc(t), 
                container.get("trunk_pos_z").acc(t));
        Eigen::Vector3d trunkAxisAnglesAcc =
            Eigen::Vector3d(
                container.get("trunk_axis_x").acc(t), 
                container.get("trunk_axis_y").acc(t), 
                container.get("trunk_axis_z").acc(t));
        Eigen::Vector3d footPosAcc = 
            Eigen::Vector3d(
                container.get("foot_pos_x").acc(t), 
                container.get("foot_pos_y").acc(t), 
                container.get("foot_pos_z").acc(t));
        //Assign model
        model.trunkFootIK(
            Leph::HumanoidFixedModel::LeftSupportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAxisAngles),
            footPos);
        //Compute joints velocities and accelerations
        //Axis differentiation is converted in proper angular
        //velocity and acceleration
        Eigen::VectorXd dq = model.trunkFootIKVel(
            trunkPosVel, 
            Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesVel), 
            footPosVel);
        Eigen::VectorXd ddq = model.trunkFootIKAcc(
            dq,
            trunkPosVel, 
            Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesVel), 
            footPosVel,
            trunkPosAcc, 
            Leph::AxisDiffToAngularDiff(trunkAxisAngles, trunkAxisAnglesAcc), 
            footPosAcc);
        //Compute static torques
        Eigen::VectorXd tau;
        if (isDoubleSupport) {
            tau = model.get().inverseDynamicsClosedLoop(
                "right_foot_tip", false, dq, ddq);
            tau(model.get().getDOFIndex("base_x")) = 0.0;
            tau(model.get().getDOFIndex("base_y")) = 0.0;
            tau(model.get().getDOFIndex("base_z")) = 0.0;
        } else {
            //Compute single support static torques
            tau = model.get().inverseDynamics(dq, ddq);
            tau(model.get().getDOFIndex("base_x")) = 0.0;
            tau(model.get().getDOFIndex("base_y")) = 0.0;
            tau(model.get().getDOFIndex("base_z")) = 0.0;
        }
        if (!isLoop) {
            plot.add(Leph::VectorLabel(
                "t", t,
                "left:hip_yaw", tau(model.get().getDOFIndex("left_hip_yaw")),
                "left:hip_pitch", tau(model.get().getDOFIndex("left_hip_pitch")),
                "left:hip_roll", tau(model.get().getDOFIndex("left_hip_roll")),
                "left:knee", tau(model.get().getDOFIndex("left_knee")),
                "left:ankle_pitch", tau(model.get().getDOFIndex("left_ankle_pitch")),
                "left:ankle_roll", tau(model.get().getDOFIndex("left_ankle_roll")),
                "right:hip_yaw", tau(model.get().getDOFIndex("right_hip_yaw")),
                "right:hip_pitch", tau(model.get().getDOFIndex("right_hip_pitch")),
                "right:hip_roll", tau(model.get().getDOFIndex("right_hip_roll")),
                "right:knee", tau(model.get().getDOFIndex("right_knee")),
                "right:ankle_pitch", tau(model.get().getDOFIndex("right_ankle_pitch")),
                "right:ankle_roll", tau(model.get().getDOFIndex("right_ankle_roll"))
            ));
        }
        //Display trunk and foot trajectory
        viewer.addTrackedPoint(
            model.get().position("trunk", "origin"), 
            Leph::ModelViewer::Purple);
        viewer.addTrackedPoint(
            model.get().position("right_foot_tip", "origin"), 
            Leph::ModelViewer::Cyan);
        //Display ZMP
        viewer.addTrackedPoint(
            model.zeroMomentPoint("origin", dq, ddq),
            Leph::ModelViewer::Yellow);
        //Draw model
        Leph::ModelDraw(model.get(), viewer);
        scheduling.wait();
        if (t >= container.max()) {
            t = 0.0;
            isLoop = true;
        }
        t += 0.02;
    }
    plot.plot("t", "left:*").render();
    plot.plot("t", "right:*").render();
}
static void displayTrajectory(Leph::ModelViewer& viewer, 
    const Eigen::VectorXd& params)
{
    displayTrajectory(viewer, generateTrajectory(params));
}

/**
 * Plot the trajectory computed from
 * given parameters
 */
static void plotTrajectory(const Leph::SplineContainer<Leph::CubicSpline>& container)
{
    Leph::Plot plot;
    for (double t=0.0;t<container.max();t+=0.02) {
        plot.add(Leph::VectorLabel(
            "time", t,
            "is_double_support", container.get("is_double_support").pos(t),
            "trunk_pos_x", container.get("trunk_pos_x").pos(t),
            "trunk_pos_y", container.get("trunk_pos_y").pos(t),
            "trunk_pos_z", container.get("trunk_pos_z").pos(t),
            "trunk_pos_x_vel", container.get("trunk_pos_x").vel(t),
            "trunk_pos_y_vel", container.get("trunk_pos_y").vel(t),
            "trunk_pos_z_vel", container.get("trunk_pos_z").vel(t)
        ));
    }
    plot.plot("time", "all").render();
    plot.clear();
    for (double t=0.0;t<container.max();t+=0.02) {
        plot.add(Leph::VectorLabel(
            "time", t,
            "trunk_axis_x", container.get("trunk_axis_x").pos(t),
            "trunk_axis_y", container.get("trunk_axis_y").pos(t),
            "trunk_axis_z", container.get("trunk_axis_z").pos(t),
            "trunk_axis_x_vel", container.get("trunk_axis_x").vel(t),
            "trunk_axis_y_vel", container.get("trunk_axis_y").vel(t),
            "trunk_axis_z_vel", container.get("trunk_axis_z").vel(t)
        ));
    }
    plot.plot("time", "all").render();
    plot.clear();
    for (double t=0.0;t<container.max();t+=0.02) {
        plot.add(Leph::VectorLabel(
            "time", t,
            "foot_pos_x", container.get("foot_pos_x").pos(t),
            "foot_pos_y", container.get("foot_pos_y").pos(t),
            "foot_pos_z", container.get("foot_pos_z").pos(t),
            "foot_pos_x_vel", container.get("foot_pos_x").vel(t),
            "foot_pos_y_vel", container.get("foot_pos_y").vel(t),
            "foot_pos_z_vel", container.get("foot_pos_z").vel(t)
        ));
    }
    plot.plot("time", "all").render();
}
static void plotTrajectory(const Eigen::VectorXd& params) 
{
    plotTrajectory(generateTrajectory(params));
}

/**
 * Save the given trajectory in spline format
 * with given name/path
 */
static void saveTrajectory(const Eigen::VectorXd& params, const std::string& filename)
{
    Leph::SplineContainer<Leph::CubicSpline> container = generateTrajectory(params);
    container.exportData(filename);
}

/**
 * Print and show information about given trajectory
 */
static void showTrajectory(const Leph::SplineContainer<Leph::CubicSpline>& container)
{
    std::cout << "Score: " << scoreTrajectory(container, true) << std::endl;
    plotTrajectory(container);
    Leph::ModelViewer viewer(1200, 900);
    displayTrajectory(viewer, container);
}
static void showTrajectory(const Eigen::VectorXd& params)
{
    showTrajectory(generateTrajectory(params));
}

/**
 * Search and optimize a dynamic trajectory
 * to reach the target
 */
static void optimizeDynamicTrajectory()
{
    //Display initial trajectory
    Eigen::VectorXd initParams = initParameters();
    //showTrajectory(initParams);
    saveTrajectory(initParams, 
        "/tmp/trajInit_" 
        + std::to_string(TrajectoryLength) 
        + ".splines");

    //Try to find minimum torques configuration with
    //CMA-ES optimization
    libcmaes::FitFuncEigen fitness = 
        [](const Eigen::VectorXd& params) 
    {
        return scoreTrajectory(params);
    };
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(initParams, -1.0, 10);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(true);
    cmaparams.set_restarts(4);
    cmaparams.set_max_iter(400);
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams);
    Eigen::VectorXd params = 
        cmasols.get_best_seen_candidate().get_x_dvec();
    double score = 
        cmasols.get_best_seen_candidate().get_fvalue();

    //Print final score and display founded trajectory
    //showTrajectory(params);
    saveTrajectory(params, 
        "/tmp/trajBest_"
        + std::to_string(TrajectoryLength) + "_" 
        + std::to_string(score)
        + ".splines");
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: ./app [static|dynamic|viewer [filename]]" << std::endl;
        return 1;
    }
    std::string mode = std::string(argv[1]);

    if (mode == "static") {
        //Compute optimal pose in single support
        //minimizing static torques
        computeOptimalPose(); 
    } else if (mode == "dynamic") {
        //Optime the dybnamic trajectory to reach
        //the static target
        optimizeDynamicTrajectory();
    } else if (mode == "viewer") {
        if (argc != 3) {
            std::cout << "Filename needed" << std::endl;
            return 1;
        }
        std::string filename = argv[2];
        //Loading
        std::cout << "Loading " << filename << std::endl;
        Leph::SplineContainer<Leph::CubicSpline> container;
        container.importData(filename);
        showTrajectory(container);
    } else {
        std::cout << "Bad mode" << std::endl;
    }

    return 0;
}

