#include <iostream>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/JointModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"
#include "Model/NamesModel.h"

#ifdef LEPH_VIEWER_ENABLED
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"
#endif

/**
 * Global CMA-ES configuration
 */
static unsigned int cmaesMaxIterations = 1000;
static unsigned int cmaesRestarts = 3;
static unsigned int cmaesLambda = 10;
static double cmaesSigma = -1.0;

/**
 * Number of splines points
 */
static unsigned int numberOptimizationPoints = 10;

/**
 * Cart names optimized
 */
static std::vector<std::string> cartNames = {
    "trunk_pos_x", "trunk_pos_y", "trunk_pos_z",
    "trunk_axis_x", "trunk_axis_y", "trunk_axis_z",
    "foot_pos_x", "foot_pos_y", "foot_pos_z",
    //"foot_axis_x", "foot_axis_y", "foot_axis_z",
};

/**
 * Split the given trajectories at given time
 * for given spline name. "All" means all trajectories.
 * True is returned if at least one point is realy added.
 */
static bool splitTrajectories(
    Leph::Trajectories& trajs, 
    double time, 
    const std::string& nameCart = "all")
{
    bool isUpdated = false;
    for (const std::string& name : cartNames) {
        if (nameCart == "all" || nameCart == name) {
            auto points = trajs.get(name).points();
            //Cancel if empty spline
            if (points.size() < 2) {
                break;
            }
            if (
                time < points.front().time || 
                time > points.back().time
            ) {
                break;
            }
            //Cancel if asked point is too close
            //from an existing points
            bool isValid = true;
            for (size_t i=0;i<points.size();i++) {
                if (fabs(time-points[i].time) <= 0.01) {
                    isValid = false;
                    break;
                }
            }
            if (!isValid) {
                break;
            }
            //Add a new spline point
            trajs.get(name).addPoint(
                time, 
                trajs.get(name).pos(time),
                trajs.get(name).vel(time),
                trajs.get(name).acc(time));
            isUpdated = true;
            //Uptade internal spline parts
            trajs.get(name).computeSplines();
        }
    }

    return isUpdated;
}

/**
 * Count the minimum number of points 
 * per trajectory
 */
static size_t countParameters(
    const Leph::Trajectories& trajsGoal)
{
    size_t minCount = (size_t)-1;
    //Count the number of parameters
    for (const std::string& name : cartNames) {
        size_t count = 0;
        const auto& points = trajsGoal.get(name).points();
        for (size_t i=1;i<points.size()-1;i++) {
            count++;
        }
        if (count < minCount || minCount == (size_t)-1) {
            minCount = count;
        }
    }

    return minCount;
}

/**
 * Return initial parameters from
 * given trajectories
 */
static Eigen::VectorXd initParameters(
    const Leph::Trajectories& trajsGoal)
{
    //Copy the trajectories
    Leph::Trajectories trajsFeed = trajsGoal;
    //Build initial spline parameters
    size_t countParams = 0;
    //Count the number of parameters
    std::cout << "Initial Parameters split:" << std::endl;
    for (const std::string& name : cartNames) {
        const auto& points = trajsFeed.get(name).points();
        std::cout << "Size: " << points.size()-2 << " ";
        std::cout << "Name: " << name << " ";
        for (size_t i=1;i<points.size()-1;i++) {
            std::cout << points[i].time << " ";
            countParams += 4;
        }
        std::cout << std::endl;
    }
    //Build vector
    Eigen::VectorXd initParams(countParams);
    countParams = 0;
    //Assign initial values
    for (const std::string& name : cartNames) {
        const auto& points = trajsFeed.get(name).points();
        for (size_t i=1;i<points.size()-1;i++) {
            initParams(countParams + 0) = points[i].position;
            initParams(countParams + 1) = points[i].velocity;
            initParams(countParams + 2) = points[i].acceleration;
            initParams(countParams + 3) = points[i].time;
            countParams += 4;
        }
    }

    return initParams;
}

/**
 * Build trajectories from given parameters and 
 * goal trajectories
 */
static Leph::Trajectories buildTrajectories(
    const Eigen::VectorXd& params, 
    const Leph::Trajectories& trajsGoal)
{
    //Copy (timing and initial/final pose) goal trajectories
    Leph::Trajectories trajsFeed = trajsGoal;
    //Assign values from parameters
    size_t indexParams = 0;
    for (const std::string& name : cartNames) {
        auto& points = trajsFeed.get(name).points();
        for (size_t i=1;i<points.size()-1;i++) {
            points[i].position = params(indexParams + 0);
            points[i].velocity = params(indexParams + 1);
            points[i].acceleration = params(indexParams + 2);
            points[i].time = params(indexParams + 3);
            indexParams += 4;
        }
        trajsFeed.get(name).computeSplines();
    }

    return trajsFeed;
}

/**
 * Return the error between built trajectories through
 * forward simulation and expected goal trajectories
 */
static double scoreTrajectories(
    const Eigen::VectorXd& params, 
    const Leph::Trajectories& trajsGoal, 
    int verboseLevel)
{
    //Initialize Humanoid models
    Leph::HumanoidModel modelSim(
        Leph::SigmabanModel, 
        "left_foot_tip", false);
    Leph::HumanoidFixedModel modelGoal(
        Leph::SigmabanModel);
    Leph::HumanoidFixedModel modelFeed(
        Leph::SigmabanModel);

    //Build trajectories
    Leph::Trajectories trajsFeed = buildTrajectories(params, trajsGoal);
    
    //Initialize simulator
    Leph::ForwardSimulation sim(modelSim);
    
#ifdef LEPH_VIEWER_ENABLED
    Leph::ModelViewer* viewer = nullptr;
    if (verboseLevel >= 2) {
        viewer = new Leph::ModelViewer(1200, 900);
    }
    Leph::Plot plot;
#endif
    
    //Main loop
    double minTime = trajsGoal.min();
    double maxTime = trajsGoal.max();
    bool isInit = true;
    double errorMax = -1.0;
    double errorCount = 0.0;
    double errorSum = 0.0;
    double errorMaxTime = 0.0;
    Eigen::VectorXd errorMaxAll(Leph::NamesDOF.size());
    for (size_t i=0;i<Leph::NamesDOF.size();i++) {
        errorMaxAll(i) = -1.0;
    }
    std::string errorMaxName = "";
    for (double t=minTime;t<=maxTime;t+=0.01) {
#ifdef LEPH_VIEWER_ENABLED
        //Viewer update
        if (verboseLevel >= 2) {
            if (!viewer->update()) {
                break;
            }
        }
#endif
        //Compute dof target for goal model
        Eigen::Vector3d trunkPosGoal;
        Eigen::Vector3d trunkAxisGoal;
        Eigen::Vector3d footPosGoal;
        Eigen::Vector3d footAxisGoal;
        bool isDoubleSupportGoal;
        Leph::HumanoidFixedModel::SupportFoot supportFootGoal;
        Leph::TrajectoriesTrunkFootPos(
            t, trajsGoal, 
            trunkPosGoal, trunkAxisGoal,
            footPosGoal, footAxisGoal);
        Leph::TrajectoriesSupportFootState(
            t, trajsGoal,
            isDoubleSupportGoal, 
            supportFootGoal);
        bool isSuccessGoal = modelGoal.trunkFootIK(
            supportFootGoal,
            trunkPosGoal,
            Leph::AxisToMatrix(trunkAxisGoal),
            footPosGoal,
            Leph::AxisToMatrix(footAxisGoal));
        if (!isSuccessGoal) {
            std::cout << "TrajsGoal IK Error t=" 
                << t << std::endl;
            exit(1);
        }
        
        //Compute dof target for feed model
        Eigen::Vector3d trunkPosFeed;
        Eigen::Vector3d trunkAxisFeed;
        Eigen::Vector3d footPosFeed;
        Eigen::Vector3d footAxisFeed;
        bool isDoubleSupportFeed;
        Leph::HumanoidFixedModel::SupportFoot supportFootFeed;
        Leph::TrajectoriesTrunkFootPos(
            t, trajsFeed, 
            trunkPosFeed, trunkAxisFeed,
            footPosFeed, footAxisFeed);
        Leph::TrajectoriesSupportFootState(
            t, trajsFeed,
            isDoubleSupportFeed, 
            supportFootFeed);
        double boundIKDistance = 0.0;
        bool isSuccessFeed = modelFeed.trunkFootIK(
            supportFootFeed,
            trunkPosFeed,
            Leph::AxisToMatrix(trunkAxisFeed),
            footPosFeed,
            Leph::AxisToMatrix(footAxisFeed),
            &boundIKDistance);
        //Cost near IK bound
        double boundIKThreshold = 1e-2;
        if (boundIKDistance < boundIKThreshold) {
            return 1000.0 + 1000.0*(boundIKThreshold - boundIKDistance);
        }
        //IK Error
        if (!isSuccessFeed) {
            return 2000.0;
        }
        
        //Initialization
        if (isInit) {
            isInit = false;
            for (const std::string& name : Leph::NamesDOF) {
                sim.positions()(modelSim.getDOFIndex(name)) = 
                    modelFeed.get().getDOF(name);
                sim.goals()(modelSim.getDOFIndex(name)) = 
                    modelFeed.get().getDOF(name);
                sim.velocities()(modelSim.getDOFIndex(name)) = 0.0;
            }
        }
        
        //Assign goal to simulation
        for (const std::string& name : Leph::NamesDOF) {
            sim.goals()(modelSim.getDOFIndex(name)) = 
                modelFeed.get().getDOF(name);
            
        }

        //Simulation step
        for (size_t k=0;k<10;k++) {
            sim.update(0.001);
        }

        //Compute error
        size_t index = 0;
        for (const std::string& name : Leph::NamesDOF) {
            double error = pow(180.0/M_PI
                *(modelGoal.get().getDOF(name) - modelSim.getDOF(name)), 2);
            errorSum += error;
            errorCount += 1.0;
            if (errorMax < 0.0 || errorMax < error) {
                errorMax = error;
                errorMaxTime = t;
                errorMaxName = name;
            }
            if (errorMaxAll(index) < 0.0 || errorMaxAll(index) < error) {
                errorMaxAll(index) = error;
            }
            index++;
#ifdef LEPH_VIEWER_ENABLED
            plot.add({
                "t", t,
                "goal:"+name, 180.0/M_PI*modelGoal.get().getDOF(name),
                "feed:"+name, 180.0/M_PI*modelFeed.get().getDOF(name),
                "sim:"+name, 180.0/M_PI*modelSim.getDOF(name),
            });
#endif
        }

#ifdef LEPH_VIEWER_ENABLED
        //Display
        if (verboseLevel >= 2) {
            Leph::ModelDraw(modelSim, *viewer, 1.0);
            Leph::ModelDraw(modelGoal.get(), *viewer, 0.6);
            Leph::ModelDraw(modelFeed.get(), *viewer, 0.4);
        }
#endif
    }
    if (verboseLevel >= 1) {
        std::cout << "MeanError: " << sqrt(errorSum/errorCount) << std::endl;
        std::cout << "MaxError: " << sqrt(errorMax) << std::endl;
        std::cout << "MaxErrorTime: " << errorMaxTime << std::endl;
        std::cout << "MaxErrorName: " << errorMaxName << std::endl;
        std::cout << "MaxAllErrorMean: " << sqrt(errorMaxAll.mean()) << std::endl;
        std::cout << "MaxAllError: " << (errorMaxAll.array().sqrt().transpose()) << std::endl;
    }

#ifdef LEPH_VIEWER_ENABLED
    if (verboseLevel >= 2) {
        delete viewer;
        for (const std::string& name : Leph::NamesDOF) {
            plot
                .plot("t", "goal:" + name)
                .plot("t", "feed:" + name)
                .plot("t", "sim:" + name)
                .render();
        }
    }
#endif

    return errorSum/errorCount;
}

/**
 * Return the error between built trajectories 
 * and feed forward generated trajectories
 * (Used to initialize the trajectories fitting)
 */
static double scoreFitting(
    const Eigen::VectorXd& params, 
    const Leph::Trajectories& trajsGoal, 
    int verboseLevel)
{
    //Initialize Humanoid models
    Leph::HumanoidFixedModel modelGoal(
        Leph::SigmabanModel);
    Leph::HumanoidFixedModel modelFeed(
        Leph::SigmabanModel);

    //Build trajectories
    Leph::Trajectories trajsFeed = buildTrajectories(params, trajsGoal);
    
#ifdef LEPH_VIEWER_ENABLED
    Leph::ModelViewer* viewer = nullptr;
    if (verboseLevel >= 2) {
        viewer = new Leph::ModelViewer(1200, 900);
    }
    Leph::Plot plot;
#endif
    
    //Main loop
    double minTime = trajsGoal.min();
    double maxTime = trajsGoal.max();
    double errorMax = -1.0;
    double errorCount = 0.0;
    double errorSum = 0.0;
    double errorMaxTime = 0.0;
    Eigen::VectorXd errorMaxAll(Leph::NamesDOF.size());
    for (size_t i=0;i<Leph::NamesDOF.size();i++) {
        errorMaxAll(i) = -1.0;
    }
    std::string errorMaxName = "";
    for (double t=minTime;t<=maxTime;t+=0.01) {
#ifdef LEPH_VIEWER_ENABLED
        //Viewer update
        if (verboseLevel >= 2) {
            if (!viewer->update()) {
                break;
            }
        }
#endif
        //Compute kinematics position, velocity and
        //acceleration on goal model
        Eigen::VectorXd dq;
        Eigen::VectorXd ddq;
        bool isSuccessGoal = Leph::TrajectoriesComputeKinematics(
            t, trajsGoal, modelGoal, dq, ddq);
        if (!isSuccessGoal) {
            std::cout << "TrajsGoal IK Error t=" 
                << t << std::endl;
            exit(1);
        }
        //Compute external torque on goal trajectories
        Eigen::VectorXd torques = modelGoal.get().inverseDynamics(dq, ddq);
        
        //Compute dof target for feed model
        Eigen::Vector3d trunkPosFeed;
        Eigen::Vector3d trunkAxisFeed;
        Eigen::Vector3d footPosFeed;
        Eigen::Vector3d footAxisFeed;
        bool isDoubleSupportFeed;
        Leph::HumanoidFixedModel::SupportFoot supportFootFeed;
        Leph::TrajectoriesTrunkFootPos(
            t, trajsFeed, 
            trunkPosFeed, trunkAxisFeed,
            footPosFeed, footAxisFeed);
        Leph::TrajectoriesSupportFootState(
            t, trajsFeed,
            isDoubleSupportFeed, 
            supportFootFeed);
        double boundIKDistance = 0.0;
        bool isSuccessFeed = modelFeed.trunkFootIK(
            supportFootFeed,
            trunkPosFeed,
            Leph::AxisToMatrix(trunkAxisFeed),
            footPosFeed,
            Leph::AxisToMatrix(footAxisFeed),
            &boundIKDistance);
        //Cost near IK bound
        double boundIKThreshold = 1e-2;
        if (boundIKDistance < boundIKThreshold) {
            return 1000.0 + 1000.0*(boundIKThreshold - boundIKDistance);
        }
        //IK Error
        if (!isSuccessFeed) {
            return 2000.0;
        }

        //Compute feed forward
        Leph::JointModel jointModel;
        for (const std::string& name : Leph::NamesDOF) {
            if (
                name.find("shoulder") != std::string::npos ||
                name.find("head") != std::string::npos ||
                name.find("elbow") != std::string::npos
            ) {
                continue;
            }
            size_t indexDOF = modelGoal.get().getDOFIndex(name);
            //Compute feed forward
            double offset = jointModel.computeFeedForward(
                dq(indexDOF), ddq(indexDOF), torques(indexDOF));
            //Assign offset
            modelGoal.get().setDOF(name, offset + modelGoal.get().getDOF(name));
        }
        
        size_t index = 0;
        for (const std::string& name : Leph::NamesDOF) {
            //Compute error
            double error = pow(180.0/M_PI
                *(modelGoal.get().getDOF(name) - modelFeed.get().getDOF(name)), 2);
            errorSum += error;
            errorCount += 1.0;
            if (errorMax < 0.0 || errorMax < error) {
                errorMax = error;
                errorMaxTime = t;
                errorMaxName = name;
            }
            if (errorMaxAll(index) < 0.0 || errorMaxAll(index) < error) {
                errorMaxAll(index) = error;
            }
            index++;
#ifdef LEPH_VIEWER_ENABLED
            plot.add({
                "t", t,
                "goal:"+name, 180.0/M_PI*modelGoal.get().getDOF(name),
                "feed:"+name, 180.0/M_PI*modelFeed.get().getDOF(name),
            });
#endif
        }

#ifdef LEPH_VIEWER_ENABLED
        //Display
        if (verboseLevel >= 2) {
            Leph::ModelDraw(modelGoal.get(), *viewer, 1.0);
            Leph::ModelDraw(modelFeed.get(), *viewer, 0.5);
        }
#endif
    }
    if (verboseLevel >= 1) {
        std::cout << "MeanError: " << sqrt(errorSum/errorCount) << std::endl;
        std::cout << "MaxError: " << sqrt(errorMax) << std::endl;
        std::cout << "MaxErrorTime: " << errorMaxTime << std::endl;
        std::cout << "MaxErrorName: " << errorMaxName << std::endl;
        std::cout << "MaxAllErrorMean: " << sqrt(errorMaxAll.mean()) << std::endl;
        std::cout << "MaxAllError: " << (errorMaxAll.array().sqrt().transpose()) << std::endl;
    }

#ifdef LEPH_VIEWER_ENABLED
    if (verboseLevel >= 2) {
        delete viewer;
        for (const std::string& name : Leph::NamesDOF) {
            plot
                .plot("t", "goal:" + name)
                .plot("t", "feed:" + name)
                .render();
        }
    }
#endif

    return errorSum/errorCount;
}

/**
 * Optimize faked Cartesian target trajectories
 * in order that simulated model matches originaly
 * expected trajectories
 */
int main(int argc, char** argv)
{
    //Parse user inputs
    if (
        (argc != 4 && argc != 5) || (
        std::string(argv[1]) != "RUN" &&
        std::string(argv[1]) != "RESTART" &&
        std::string(argv[1]) != "RUN_FEED" &&
        std::string(argv[1]) != "RESTART_FEED")
    ) {
        std::cout << "Usage ./app [RUN] [inGoalSplines] [outCorrectedSplines]" << std::endl;
        std::cout << "Usage ./app [RESTART] [inGoalSplines] [inRestartSplines] [outCorrectedSplines]" << std::endl;
        std::cout << "Usage ./app [RUN_FEED] [inGoalSplines] [outCorrectedSplines]" << std::endl;
        std::cout << "Usage ./app [RESTART_FEED] [inGoalSplines] [inRestartSplines] [outCorrectedSplines]" << std::endl;
        return 1;
    }
    std::string goalFilename = "";
    std::string restartFilename = "";
    std::string outFilename = "";
    bool isFeedForward = false;
    if (std::string(argv[1]) == "RUN") {
        isFeedForward = false;
        goalFilename = argv[2];
        restartFilename = "";
        outFilename = argv[3];
    }
    if (std::string(argv[1]) == "RESTART") {
        isFeedForward = false;
        goalFilename = argv[2];
        restartFilename = argv[3];
        outFilename = argv[4];
    }
    if (std::string(argv[1]) == "RUN_FEED") {
        isFeedForward = true;
        goalFilename = argv[2];
        restartFilename = "";
        outFilename = argv[3];
    }
    if (std::string(argv[1]) == "RESTART_FEED") {
        isFeedForward = true;
        goalFilename = argv[2];
        restartFilename = argv[3];
        outFilename = argv[4];
    }

    //Load trajectories from file
    std::cout << "Loading goal: " << goalFilename << std::endl;
    Leph::Trajectories trajsGoal;
    trajsGoal.importData(goalFilename);

    //Verbose
    if (isFeedForward) {
        std::cout << "Minimizing feed forward distance." << std::endl;
    } else {
        std::cout << "Minimizing simulated distance." << std::endl;
    }
    
    //Subdivise the trajectories
    if (numberOptimizationPoints != 0) {
        while (true) {
            size_t count = countParameters(trajsGoal);
            if (count >= numberOptimizationPoints) {
                break;
            }
            double splitTime = 0.0;
            double maxLength = 0.0;
            const auto& points = trajsGoal.get(cartNames.front()).points();
            for (size_t i=1;i<points.size();i++) {
                double length = points[i].time - points[i-1].time;
                if (length > maxLength) {
                    maxLength = length;
                    splitTime = 0.5*points[i].time + 0.5*points[i-1].time;
                }
            }
            std::cout << "Split trajectories at t=" << splitTime << std::endl;
            splitTrajectories(trajsGoal, splitTime, "all");
        }
    }

    //Build initial parameters
    Eigen::VectorXd initParams;
    if (restartFilename != "") {
        //Load restarting data
        std::cout << "Loading restart: " << restartFilename << std::endl;
        Leph::Trajectories trajsRestart;
        trajsRestart.importData(restartFilename);
        initParams = initParameters(trajsRestart);
    } else {
        initParams = initParameters(trajsGoal);
    }
    
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&trajsGoal, &isFeedForward](const Eigen::VectorXd& params) 
    {
        double cost = 0.0;
        try {
            if (isFeedForward) {
                cost += scoreFitting(params, trajsGoal, 0);
            } else {
                cost += scoreTrajectories(params, trajsGoal, 0);
            }
        } catch (const std::runtime_error& e) {
            cost += 10000.0;
        } catch (const std::logic_error& e) {
            //Catch exception only for unbounded angle
            if (
                std::string(e.what()).find("AxisAngle unbounded angle") 
                == std::string::npos
            ) {
                throw;
            } else {
                cost += 10000.0;
            }
        }
        return cost;
    };
    
    Eigen::VectorXd bestParams = initParams;
    double bestScore = -1.0;
    int iteration = 1;
    
    //Progress function
    libcmaes::ProgressFunc<
        libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress = 
        [&trajsGoal, &bestParams, &bestScore, &iteration, &outFilename, &isFeedForward](
            const libcmaes::CMAParameters<>& cmaparams, 
            const libcmaes::CMASolutions& cmasols)
    {
        //Retrieve best Trajectories and score
        Eigen::VectorXd params = 
            cmasols.get_best_seen_candidate().get_x_dvec();
        double score = 
            cmasols.get_best_seen_candidate().get_fvalue();
        if (!std::isnan(score) && (bestScore < 0.0 || bestScore > score)) {
            bestParams = params;
            bestScore = score;
        }
        if (iteration % 50 == 0) {
            std::cout << "============" << std::endl;
            std::cout << "Dimension: " << params.size() << std::endl;
            std::cout << "Saving to: " << outFilename << std::endl;
            std::cout << "BestScore: " << bestScore << std::endl;
            std::cout << "BestParams: ";
            for (size_t i=0;i<(size_t)bestParams.size();i++) {
                std::cout << std::setprecision(10) << bestParams(i);
                if (i != (size_t)bestParams.size()-1) {
                    std::cout << ", ";
                } else {
                    std::cout << ";" << std::endl;
                }
            }
            std::cout << "Score: " << score<< std::endl;
            std::cout << "Params: " << params.transpose() << std::endl;
            std::cout << "============" << std::endl;
            if (isFeedForward) {
                scoreFitting(bestParams, trajsGoal, 1);
            } else {
                scoreTrajectories(bestParams, trajsGoal, 1);
            }
            std::cout << "============" << std::endl;
            Leph::Trajectories bestTrajs = buildTrajectories(bestParams, trajsGoal);  
            bestTrajs.exportData(outFilename);
        }
        iteration++;
        
        //Call default CMA-ES default progress function
	return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
            ::_defaultPFunc(cmaparams, cmasols);
    };

    //Verbose initial score
    if (isFeedForward) {
        double initScore = scoreFitting(initParams, trajsGoal, 2);
        std::cout << "InitScore: " << initScore << std::endl;
    } else {
        double initScore = scoreTrajectories(initParams, trajsGoal, 2);
        std::cout << "InitScore: " << initScore << std::endl;
    }
    
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(initParams, 
        cmaesSigma, cmaesLambda);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(1);
    cmaparams.set_restarts(cmaesRestarts);
    cmaparams.set_max_iter(cmaesMaxIterations);
    cmaparams.set_ftolerance(1e-9);
    
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams, progress);
    
    //Retrieve best Trajectories and score
    bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
    bestScore = cmasols.get_best_seen_candidate().get_fvalue();
    Leph::Trajectories bestTrajs = buildTrajectories(bestParams, trajsGoal);  
    bestTrajs.exportData(outFilename);
    
    //Show best found parameters
    if (isFeedForward) {
        scoreFitting(bestParams, trajsGoal, 2);
    } else {
        scoreTrajectories(bestParams, trajsGoal, 2);
    }
    std::cout << "BestParams: " << bestParams.transpose() << std::endl;
    std::cout << "BestScore: " << bestScore << std::endl;

    return 0;
}

