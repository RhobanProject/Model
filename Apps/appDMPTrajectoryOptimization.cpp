#include <iostream>
#include <thread>
#include <chrono>
#include <libcmaes/cmaes.h>
#include "DMP/DMPSpline.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Spline/SplineContainer.hpp"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"

/**
 * Configuration
 */
static unsigned int kernelNum = 8;
static double overlap = 0.2;
static double maxTimeStep = 0.005;
static double timeLength = 3.0;
static double kickTime = 1.5;
static double kickPosX = 0.01;
static double kickPosY = -0.11;
static double kickPosZ = 0.07;
static double kickVelX = 0.3;

/**
 * Return initial parameterd
 */
static Eigen::VectorXd initParameters()
{
    Eigen::VectorXd params = Eigen::VectorXd::Zero(9*kernelNum+1);
    //Eigen::VectorXd params = Eigen::VectorXd::Zero(2*9*kernelNum+1);
    params(9*kernelNum) = 100.0*0.2;
    return params;
}

/**
 * Build DMP Spline trajectories from given parameters
 */
static Leph::SplineContainer<Leph::DMPSpline> buildTrajectories(const Eigen::VectorXd& params)
{
    Leph::SplineContainer<Leph::DMPSpline> traj;

    Eigen::Vector3d staticTrunkPos(-0.00557785331559037,  -0.0115849568418458, 0.28);
    Eigen::Vector3d staticTrunkAngles(-0.672036398746933, 0.0743358280850477, 0.0028323027017884);
    Eigen::Vector3d staticFootPos(0.0208647084129351, -0.095, 0.0591693358237435);

    traj.add("trunk_pos_x", kernelNum, overlap, maxTimeStep);
    traj.add("trunk_pos_y", kernelNum, overlap, maxTimeStep);
    traj.add("trunk_pos_z", kernelNum, overlap, maxTimeStep);
    traj.add("trunk_axis_x", kernelNum, overlap, maxTimeStep);
    traj.add("trunk_axis_y", kernelNum, overlap, maxTimeStep);
    traj.add("trunk_axis_z", kernelNum, overlap, maxTimeStep);
    traj.add("foot_pos_x", kernelNum, overlap, maxTimeStep);
    traj.add("foot_pos_y", kernelNum, overlap, maxTimeStep);
    traj.add("foot_pos_z", kernelNum, overlap, maxTimeStep);

    traj.get("trunk_pos_x").addPoint(0.0, staticTrunkPos.x());
    traj.get("trunk_pos_y").addPoint(0.0, staticTrunkPos.y());
    traj.get("trunk_pos_z").addPoint(0.0, staticTrunkPos.z());
    traj.get("trunk_axis_x").addPoint(0.0, staticTrunkAngles.x());
    traj.get("trunk_axis_y").addPoint(0.0, staticTrunkAngles.y());
    traj.get("trunk_axis_z").addPoint(0.0, staticTrunkAngles.z());
    traj.get("foot_pos_x").addPoint(0.0, staticFootPos.x());
    traj.get("foot_pos_y").addPoint(0.0, staticFootPos.y());
    traj.get("foot_pos_z").addPoint(0.0, staticFootPos.z());
    
    double tmpKickVelX = params(9*kernelNum)/100.0;
    traj.get("foot_pos_x").addPoint(kickTime, kickPosX, tmpKickVelX, 0.0);
    traj.get("foot_pos_y").addPoint(kickTime, kickPosY, 0.0, 0.0);
    traj.get("foot_pos_z").addPoint(kickTime, kickPosZ, 0.0, 0.0);
    
    traj.get("trunk_pos_x").addPoint(timeLength, staticTrunkPos.x());
    traj.get("trunk_pos_y").addPoint(timeLength, staticTrunkPos.y());
    traj.get("trunk_pos_z").addPoint(timeLength, staticTrunkPos.z());
    traj.get("trunk_axis_x").addPoint(timeLength, staticTrunkAngles.x());
    traj.get("trunk_axis_y").addPoint(timeLength, staticTrunkAngles.y());
    traj.get("trunk_axis_z").addPoint(timeLength, staticTrunkAngles.z());
    traj.get("foot_pos_x").addPoint(timeLength, staticFootPos.x());
    traj.get("foot_pos_y").addPoint(timeLength, staticFootPos.y());
    traj.get("foot_pos_z").addPoint(timeLength, staticFootPos.z());

    traj.get("trunk_pos_x").setKernelWeights(params.segment(0*kernelNum, kernelNum));
    traj.get("trunk_pos_y").setKernelWeights(params.segment(1*kernelNum, kernelNum));
    traj.get("trunk_pos_z").setKernelWeights(params.segment(2*kernelNum, kernelNum));
    traj.get("trunk_axis_x").setKernelWeights(params.segment(3*kernelNum, kernelNum));
    traj.get("trunk_axis_y").setKernelWeights(params.segment(4*kernelNum, kernelNum));
    traj.get("trunk_axis_z").setKernelWeights(params.segment(5*kernelNum, kernelNum));
    traj.get("foot_pos_x").setKernelWeights(params.segment(6*kernelNum, kernelNum));
    traj.get("foot_pos_y").setKernelWeights(params.segment(7*kernelNum, kernelNum));
    traj.get("foot_pos_z").setKernelWeights(params.segment(8*kernelNum, kernelNum));
    
    /*
    traj.get("trunk_pos_x").setKernelWidths(params.segment(9*kernelNum, kernelNum));
    traj.get("trunk_pos_y").setKernelWidths(params.segment(10*kernelNum, kernelNum));
    traj.get("trunk_pos_z").setKernelWidths(params.segment(11*kernelNum, kernelNum));
    traj.get("trunk_axis_x").setKernelWidths(params.segment(12*kernelNum, kernelNum));
    traj.get("trunk_axis_y").setKernelWidths(params.segment(13*kernelNum, kernelNum));
    traj.get("trunk_axis_z").setKernelWidths(params.segment(14*kernelNum, kernelNum));
    traj.get("foot_pos_x").setKernelWidths(params.segment(15*kernelNum, kernelNum));
    traj.get("foot_pos_y").setKernelWidths(params.segment(16*kernelNum, kernelNum));
    traj.get("foot_pos_z").setKernelWidths(params.segment(17*kernelNum, kernelNum));
    */

    return traj;
}

/**
 * Score the given DMP trajectory using Robot Model
 */
static double scoreTrajectories(Leph::SplineContainer<Leph::DMPSpline>& traj, bool verbose)
{
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    double min = traj.min();
    double max = traj.max();

    Leph::ModelViewer* viewer = nullptr;
    if (verbose) {
        viewer = new Leph::ModelViewer(1200, 900);
    }

    double score = 0.0;
    for (double t=min;t<=max;t+=0.01) {
        Eigen::Vector3d trunkPos;
        Eigen::Vector3d trunkAngles;
        Eigen::Vector3d footPos;
        trunkPos.x() = traj.get("trunk_pos_x").pos(t);
        trunkPos.y() = traj.get("trunk_pos_y").pos(t);
        trunkPos.z() = traj.get("trunk_pos_z").pos(t);
        trunkAngles.x() = traj.get("trunk_axis_x").pos(t);
        trunkAngles.y() = traj.get("trunk_axis_y").pos(t);
        trunkAngles.z() = traj.get("trunk_axis_z").pos(t);
        footPos.x() = traj.get("foot_pos_x").pos(t);
        footPos.y() = traj.get("foot_pos_y").pos(t);
        footPos.z() = traj.get("foot_pos_z").pos(t);
        Eigen::Vector3d trunkPosVel;
        Eigen::Vector3d trunkAnglesVel;
        Eigen::Vector3d footPosVel;
        trunkPosVel.x() = traj.get("trunk_pos_x").vel(t);
        trunkPosVel.y() = traj.get("trunk_pos_y").vel(t);
        trunkPosVel.z() = traj.get("trunk_pos_z").vel(t);
        trunkAnglesVel.x() = traj.get("trunk_axis_x").vel(t);
        trunkAnglesVel.y() = traj.get("trunk_axis_y").vel(t);
        trunkAnglesVel.z() = traj.get("trunk_axis_z").vel(t);
        footPosVel.x() = traj.get("foot_pos_x").vel(t);
        footPosVel.y() = traj.get("foot_pos_y").vel(t);
        footPosVel.z() = traj.get("foot_pos_z").vel(t);
        Eigen::Vector3d trunkPosAcc;
        Eigen::Vector3d trunkAnglesAcc;
        Eigen::Vector3d footPosAcc;
        trunkPosAcc.x() = traj.get("trunk_pos_x").acc(t);
        trunkPosAcc.y() = traj.get("trunk_pos_y").acc(t);
        trunkPosAcc.z() = traj.get("trunk_pos_z").acc(t);
        trunkAnglesAcc.x() = traj.get("trunk_axis_x").acc(t);
        trunkAnglesAcc.y() = traj.get("trunk_axis_y").acc(t);
        trunkAnglesAcc.z() = traj.get("trunk_axis_z").acc(t);
        footPosAcc.x() = traj.get("foot_pos_x").acc(t);
        footPosAcc.y() = traj.get("foot_pos_y").acc(t);
        footPosAcc.z() = traj.get("foot_pos_z").acc(t);
        Eigen::Vector3d footAngles = Eigen::Vector3d::Zero();
        Eigen::Vector3d footAnglesVel = Eigen::Vector3d::Zero();
        Eigen::Vector3d footAnglesAcc = Eigen::Vector3d::Zero();
        
        bool isSuccess = model.trunkFootIK(
            Leph::HumanoidFixedModel::LeftSupportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAngles),
            footPos,
            Leph::AxisToMatrix(footAngles));
        if (!isSuccess) {
            score += 1000.0;
            continue;
        }
        //Axis differentiation is converted in proper angular
        //velocity and acceleration
        //Compute DOF velocities
        Eigen::VectorXd dq = model.trunkFootIKVel(
            trunkPosVel, 
            Leph::AxisDiffToAngularDiff(trunkAngles, trunkAnglesVel), 
            footPosVel,
            Leph::AxisDiffToAngularDiff(footAngles, footAnglesVel));
        //Compute DOF accelerations
        Eigen::VectorXd ddq = model.trunkFootIKAcc(
            dq,
            trunkPosVel, 
            Leph::AxisDiffToAngularDiff(trunkAngles, trunkAnglesVel), 
            footPosVel,
            Leph::AxisDiffToAngularDiff(footAngles, footAnglesVel), 
            trunkPosAcc, 
            Leph::AxisDiffToAngularDiff(trunkAngles, trunkAnglesAcc), 
            footPosAcc,
            Leph::AxisDiffToAngularDiff(footAngles, footAnglesAcc));
        
        Eigen::VectorXd torques = model.get().inverseDynamics(dq, ddq);
        torques(model.get().getDOFIndex("base_x")) = 0.0;
        torques(model.get().getDOFIndex("base_y")) = 0.0;
        torques(model.get().getDOFIndex("base_z")) = 0.0;
        torques(model.get().getDOFIndex("base_yaw")) = 0.0;
        torques(model.get().getDOFIndex("base_pitch")) = 0.0;
        torques(model.get().getDOFIndex("base_roll")) = 0.0;
        score += 0.1*torques.norm();

        if (verbose) {
            Leph::ModelDraw(model.get(), *viewer);
            viewer->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (verbose) {
        delete viewer;
    }

    return score;
}

/**
 * Simple trajectories generation
 * test using DMPSpline for static 
 * kick movement
 */
int main()
{
    Eigen::VectorXd initParams = initParameters();
    Leph::SplineContainer<Leph::DMPSpline> traj = buildTrajectories(initParams);
    traj.plot()
        .plot("t", "pos:trunk_pos_x")
        .plot("t", "pos:trunk_pos_y")
        .plot("t", "pos:trunk_pos_z")
        .plot("t", "vel:trunk_pos_x")
        .plot("t", "vel:trunk_pos_y")
        .plot("t", "vel:trunk_pos_z")
        .render();
    traj.plot()
        .plot("t", "pos:foot_pos_x")
        .plot("t", "pos:foot_pos_y")
        .plot("t", "pos:foot_pos_z")
        .plot("t", "vel:foot_pos_x")
        .plot("t", "vel:foot_pos_y")
        .plot("t", "vel:foot_pos_z")
        .render();
    traj.plot()
        .plot("t", "forcing:trunk_pos_x")
        .plot("t", "forcing:trunk_pos_y")
        .plot("t", "forcing:trunk_pos_z")
        .plot("t", "forcing:foot_pos_x")
        .plot("t", "forcing:foot_pos_y")
        .plot("t", "forcing:foot_pos_z")
        .render();
    double score = scoreTrajectories(traj, true);
    std::cout << "InitialScore=" << score 
        << " dimension=" << initParams.size() << std::endl;
    
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [](const Eigen::VectorXd& params) 
    {
        Eigen::VectorXd tmpParams = params;
        if (tmpParams(9*kernelNum) <= 0.1) {
            tmpParams(9*kernelNum) = 0.1;
            return 1000.0 + -1000.0*params(9*kernelNum);
        }
        Leph::SplineContainer<Leph::DMPSpline> traj = buildTrajectories(tmpParams);
        double scoreTraj = scoreTrajectories(traj, false);
        double scoreVel = 100.0*20.0/tmpParams(9*kernelNum);
        return scoreTraj + scoreVel;
    };
    
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(initParams, 10.0, 10);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(true);
    cmaparams.set_restarts(1);
    cmaparams.set_max_iter(1000);
    
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams);
    
    //Retrieve best Trajectories and score
    Eigen::VectorXd bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
    double bestScore = cmasols.get_best_seen_candidate().get_fvalue();
    std::cout << "BestScore: " << bestScore << std::endl;
    std::cout << "BestParams: " << bestParams.transpose() << std::endl;
    Leph::SplineContainer<Leph::DMPSpline> bestTraj = buildTrajectories(bestParams);
    bestTraj.plot()
        .plot("t", "pos:trunk_pos_x")
        .plot("t", "pos:trunk_pos_y")
        .plot("t", "pos:trunk_pos_z")
        .plot("t", "vel:trunk_pos_x")
        .plot("t", "vel:trunk_pos_y")
        .plot("t", "vel:trunk_pos_z")
        .render();
    bestTraj.plot()
        .plot("t", "pos:foot_pos_x")
        .plot("t", "pos:foot_pos_y")
        .plot("t", "pos:foot_pos_z")
        .plot("t", "vel:foot_pos_x")
        .plot("t", "vel:foot_pos_y")
        .plot("t", "vel:foot_pos_z")
        .render();
    bestTraj.plot()
        .plot("t", "forcing:trunk_pos_x")
        .plot("t", "forcing:trunk_pos_y")
        .plot("t", "forcing:trunk_pos_z")
        .plot("t", "forcing:foot_pos_x")
        .plot("t", "forcing:foot_pos_y")
        .plot("t", "forcing:foot_pos_z")
        .render();
    while (true) {
        scoreTrajectories(bestTraj, true);
    }

    return 0;
}

