#include <iostream>
#include <string>
#include <cmath>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/RBDLRootUpdate.h"
#include "Types/MapSeries.hpp"
#include "Plot/Plot.hpp"

/**
 * DOF names
 */
static std::vector<std::string> dofsNames = {
    "head_pitch", "head_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
};
static std::vector<std::string> basesNames = {
    "base_x", "base_y", "base_z",
    "base_roll", "base_pitch", "base_yaw",
};

/**
 * Global inertia default data
 * and name
 */
static Eigen::MatrixXd defaultInertiaData;
static std::map<std::string, size_t> defaultInertiaName;

/**
 * Global joint parameters size
 */
static size_t sizeJointParameters;

/**
 * Global configuration is inertia optimized
 */
bool isOptimizationInertia = false;

/**
 * Load and assign default inertia form model
 */
static void loadDefaultInertiaData()
{
    Leph::HumanoidModel model(
        Leph::SigmabanModel, 
        "left_foot_tip", false);
    defaultInertiaData = model.getInertiaData();
    defaultInertiaName = model.getInertiaName();
}

static double scoreFitness(const std::string& filename, const Eigen::VectorXd& parameters, bool verbose)
{
    //Load data into MapSeries
    Leph::MapSeries logs;
    logs.importData(filename);
    //Print statistics
    if (verbose) {
        std::cout << "Loaded " 
            << filename << ": "
            << logs.dimension() << " series from " 
            << logs.timeMin() << "s to " 
            << logs.timeMax() << "s with length "
            << logs.timeMax()-logs.timeMin() << "s" << std::endl;
    }
    double min = logs.timeMin();
    double max = logs.timeMax();

    //Assign inertia data
    Eigen::MatrixXd currentInertiaData = defaultInertiaData;
    if (isOptimizationInertia) {
        currentInertiaData.row(defaultInertiaName.at("left_elbow")) = 
            parameters.segment(sizeJointParameters + 0*10, 10).transpose();
        currentInertiaData.row(defaultInertiaName.at("left_shoulder_roll")) = 
            parameters.segment(sizeJointParameters + 1*10, 10).transpose();
        currentInertiaData.row(defaultInertiaName.at("left_shoulder_pitch")) = 
            parameters.segment(sizeJointParameters + 2*10, 10).transpose();
        currentInertiaData.row(defaultInertiaName.at("trunk")) = 
            parameters.segment(sizeJointParameters + 3*10, 10).transpose();
        //Check positive inertia parameters
        double tmpCost = 0.0;
        for (size_t i=0;i<(size_t)currentInertiaData.rows();i++) {
            //0 Mass
            if (currentInertiaData(i, 0) <= 0.0) {
                tmpCost += 1000.0 - 1000.0*currentInertiaData(i, 0);
            }
            //4 Ixx
            if (currentInertiaData(i, 4) <= 0.0) {
                tmpCost += 1000.0 - 1000.0*currentInertiaData(i, 4);
            }
            //7 Iyy
            if (currentInertiaData(i, 7) <= 0.0) {
                tmpCost += 1000.0 - 1000.0*currentInertiaData(i, 7);
            }
            //9 Izz
            if (currentInertiaData(i, 9) <= 0.0) {
                tmpCost += 1000.0 - 1000.0*currentInertiaData(i, 9);
            }
        }
        if (tmpCost > 0.0) {
            return tmpCost;
        }
    }
    
    //Full humanoid model
    Leph::HumanoidModel modelSim(
        Leph::SigmabanModel, 
        "left_foot_tip", false,
        currentInertiaData,
        defaultInertiaName);
    Leph::HumanoidModel modelRead(
        Leph::SigmabanModel, 
        "left_foot_tip", false);

    //Simulator
    Leph::ForwardSimulation sim(modelSim);
    sim.setJointModelParameters(parameters.segment(0, sizeJointParameters));
    //Initialization
    for (const std::string& name : dofsNames) {
        sim.positions()(modelSim.getDOFIndex(name)) = 
            logs.get("read:" + name, min);
        sim.goals()(modelSim.getDOFIndex(name)) = 
            logs.get("read:" + name, min);
        sim.velocities()(modelSim.getDOFIndex(name)) = 0.0;
    }
    
    //Right foot contact
    Leph::RBDL::ConstraintSet constraints;
    constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
    /*
    //Z 0
    constraints.AddConstraint(
        modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_1")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
    //X 1
    constraints.AddConstraint(
        modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_1")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
    //Y 2
    constraints.AddConstraint(
        modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_1")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));
    //Z 3
    constraints.AddConstraint(
        modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_2")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
    //X 4
    constraints.AddConstraint(
        modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_2")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
    //Z 5
    constraints.AddConstraint(
        modelSim.frameIndexToBodyId(modelSim.getFrameIndex("right_cleat_3")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
    */
    constraints.Bind(modelSim.getRBDLModel());
    sim.computeImpulses(constraints);
    
    //Main loop
    double cost = 0.0;
    Leph::VectorLabel sumError;
    Leph::VectorLabel maxError;
    for (const std::string& name : dofsNames) {
        sumError.setOrAppend(name, 0.0);
        maxError.setOrAppend(name, -1.0);
    }
    double count = 0.0;
    Leph::ModelViewer* viewer = nullptr;
    if (verbose) {
        viewer = new Leph::ModelViewer(1200, 900);
    }
    Leph::Plot plot;
    for (double t=min;t<max;t+=0.01) {
        if (verbose) {
            if (!viewer->update()) {
                break;
            }
        }
        for (const std::string& name : dofsNames) {
            sim.goals()(modelSim.getDOFIndex(name)) = 
                logs.get("goal:" + name, t);
            modelRead.setDOF(name, logs.get("read:" + name, t));
        }
        for (int k=0;k<10;k++) {
            sim.update(0.001, &constraints);
        }
        Leph::VectorLabel vect;
        for (const std::string& name : dofsNames) {
            double error = 
                sim.positions()(modelSim.getDOFIndex(name)) 
                - logs.get("read:" + name, t);
            error = pow(error*180.0/M_PI, 2);
            sumError(name) += error;
            if (maxError(name) < 0.0 || maxError(name) < error) {
                maxError(name) = error;
            }
            vect.setOrAppend("t", t);
            vect.setOrAppend("read:" + name, logs.get("read:" + name, t));
            vect.setOrAppend("goal:" + name, logs.get("goal:" + name, t));
            vect.setOrAppend("sim:" + name, sim.positions()(modelSim.getDOFIndex(name)));
            vect.setOrAppend("error:" + name, error);
        }
        count += 1.0;
        plot.add(vect);
        if (verbose) {
            Leph::ModelDraw(modelSim, *viewer);
            Leph::ModelDraw(modelRead, *viewer);
        }
    }
    
    sumError.divOp(count);
    cost = 0.2*sumError.mean() + 0.8*maxError.mean();
    cost = sqrt(cost);
    
    if (verbose) {
        delete viewer;
        sumError.sqrtOp();
        maxError.sqrtOp();
        std::cout << "MeanError:" << std::endl;
        std::cout << sumError;
        std::cout << "MaxError:" << std::endl;
        std::cout << maxError;
        std::cout << "Cost: " << cost << std::endl;
        plot
            .plot("t", "read:left_ankle_roll")
            .plot("t", "read:left_ankle_pitch")
            .plot("t", "read:left_knee")
            .plot("t", "read:left_hip_pitch")
            .plot("t", "read:left_hip_roll")
            .plot("t", "read:left_hip_yaw")
            .plot("t", "goal:left_ankle_roll")
            .plot("t", "goal:left_ankle_pitch")
            .plot("t", "goal:left_knee")
            .plot("t", "goal:left_hip_pitch")
            .plot("t", "goal:left_hip_roll")
            .plot("t", "goal:left_hip_yaw")
            .plot("t", "sim:left_ankle_roll")
            .plot("t", "sim:left_ankle_pitch")
            .plot("t", "sim:left_knee")
            .plot("t", "sim:left_hip_pitch")
            .plot("t", "sim:left_hip_roll")
            .plot("t", "sim:left_hip_yaw")
            .render();
        plot
            .plot("t", "read:right_ankle_roll")
            .plot("t", "read:right_ankle_pitch")
            .plot("t", "read:right_knee")
            .plot("t", "read:right_hip_pitch")
            .plot("t", "read:right_hip_roll")
            .plot("t", "read:right_hip_yaw")
            .plot("t", "goal:right_ankle_roll")
            .plot("t", "goal:right_ankle_pitch")
            .plot("t", "goal:right_knee")
            .plot("t", "goal:right_hip_pitch")
            .plot("t", "goal:right_hip_roll")
            .plot("t", "goal:right_hip_yaw")
            .plot("t", "sim:right_ankle_roll")
            .plot("t", "sim:right_ankle_pitch")
            .plot("t", "sim:right_knee")
            .plot("t", "sim:right_hip_pitch")
            .plot("t", "sim:right_hip_roll")
            .plot("t", "sim:right_hip_yaw")
            .render();
    }

    if (std::isnan(cost)) {
        std::cout << "NaN cost for parameters:" << parameters.transpose() << std::endl;
    }
    return cost;
}

int main()
{
    //Load default inertia data from model
    loadDefaultInertiaData();

    //Load data into MapSeries
    std::vector<std::string> filenames = {
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-07-37.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-08-03.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-08-19.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-08-40.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_x_2016-10-30-17-09-06.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_y_2016-10-30-17-10-58.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_foot_y_2016-10-30-17-11-12.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-03-37.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-03-56.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-04-14.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-04-33.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-04-56.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_shoulder_pitch_2016-10-30-17-07-02.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_x_2016-10-30-17-09-24.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_x_2016-10-30-17-09-44.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_x_2016-10-30-17-10-06.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_y_2016-10-30-17-10-19.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_y_2016-10-30-17-10-31.mapseries",
        "../../These/Data/logs-2016-10-30_model_calibration/nofall/calibration_log_trunk_y_2016-10-30-17-10-45.mapseries",
    };

    //Initialize joint model parameters
    Leph::JointModel tmpJoint;
    Eigen::VectorXd initParams = tmpJoint.getParameters();
    sizeJointParameters = initParams.size();

    if (isOptimizationInertia) {
        //Assign initial inertia parameters
        initParams.conservativeResize(sizeJointParameters + 4*10);
        initParams.segment(sizeJointParameters + 0*10, 10) = 
            defaultInertiaData.row(defaultInertiaName.at("left_elbow")).transpose();
        initParams.segment(sizeJointParameters + 1*10, 10) = 
            defaultInertiaData.row(defaultInertiaName.at("left_shoulder_roll")).transpose();
        initParams.segment(sizeJointParameters + 2*10, 10) = 
            defaultInertiaData.row(defaultInertiaName.at("left_shoulder_pitch")).transpose();
        initParams.segment(sizeJointParameters + 3*10, 10) = 
            defaultInertiaData.row(defaultInertiaName.at("trunk")).transpose();
    }
    
    //Normalization coefficient
    Eigen::VectorXd coef = initParams;
    Eigen::VectorXd coefInv = initParams;
    for (size_t i=0;i<(size_t)coef.size();i++) {
        if (fabs(coef(i)) < 1e-10) {
            coef(i) = 1e-10;
        }
        coefInv(i) = 1.0/coef(i);
    }

    //Initial verbose
    for (size_t i=0;i<filenames.size();i++) {
        double initScore = scoreFitness(filenames[i], initParams, false);
        std::cout << "InitScore=" << initScore << std::endl;
    }

    //Normalization of parameters
    initParams = coefInv.array() * initParams.array();
    Eigen::VectorXd bestParams = initParams;
    double bestScore = -1.0;
    int iteration = 1;
    
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&filenames, &coef](const Eigen::VectorXd& params) 
    {
        //Check positive parameters
        double cost = 0.0;
        bool isValide = true;
        for (size_t i=0;i<(size_t)params.size();i++) {
            if (i < sizeJointParameters && params(i) < 0.0) {
                cost += 1000.0 - 1000.0*params(i);
                isValide = false;
            }
        }
        if (!isValide) {
            return cost;
        }
        //Iterate over on all logs
        for (size_t i=0;i<filenames.size();i++) {
            try {
                cost += scoreFitness(filenames[i], coef.array() * params.array(), false);
            } catch (const std::runtime_error& e) {
                cost += 10000.0;
            }
        }
        return cost/(double)filenames.size();
    };
    
    //Progress function
    libcmaes::ProgressFunc<
        libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress = 
        [&bestParams, &bestScore, &iteration, &coef](
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
            std::cout << "BestScore: " << bestScore << std::endl;
            std::cout << "BestParams: " << (bestParams.array() * coef.array()).transpose() << std::endl;
            std::cout << "BestCoef: " << bestParams.transpose() << std::endl;
            std::cout << "Score: " << score<< std::endl;
            std::cout << "Params: " << (params.array() * coef.array()).transpose() << std::endl;
            std::cout << "Coef: " << params.transpose() << std::endl;
            std::cout << "============" << std::endl;
        }
        iteration++;
        
        //Call default CMA-ES default progress function
	return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
            ::_defaultPFunc(cmaparams, cmasols);
    };
    
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(initParams, 0.5, 10);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(false);
    cmaparams.set_restarts(2);
    cmaparams.set_max_iter(300);
    
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams, progress);
    
    //Retrieve best Trajectories and score
    bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
    bestScore = cmasols.get_best_seen_candidate().get_fvalue();
    std::cout << "BestScore: " << bestScore << std::endl;
    std::cout << "BestParams: " << (coef.array() * bestParams.array()).transpose() << std::endl;
    
    //Final verbose
    for (size_t i=0;i<filenames.size();i++) {
        double finalScore = scoreFitness(filenames[i], coef.array() * bestParams.array(), true);
        std::cout << "FinalScore=" << finalScore << std::endl;
    }

    return 0;
}

