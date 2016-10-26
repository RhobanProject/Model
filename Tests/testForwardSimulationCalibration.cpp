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
    currentInertiaData.row(defaultInertiaName.at("left_elbow")) = 
        parameters.segment(sizeJointParameters + 0*10, 10).transpose();
    currentInertiaData.row(defaultInertiaName.at("left_shoulder_roll")) = 
        parameters.segment(sizeJointParameters + 1*10, 10).transpose();
    currentInertiaData.row(defaultInertiaName.at("left_shoulder_pitch")) = 
        parameters.segment(sizeJointParameters + 2*10, 10).transpose();
    currentInertiaData.row(defaultInertiaName.at("trunk")) = 
        parameters.segment(sizeJointParameters + 3*10, 10).transpose();
    
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
    constraints.Bind(modelSim.getRBDLModel());
    sim.computeImpulses(constraints);
    
    //Main loop
    double cost = 0.0;
    double sumError = 0.0;
    double maxError = -1.0;
    double maxErrorTime = 0.0;
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
        double error = 
            sim.positions()(modelSim.getDOFIndex("left_shoulder_pitch")) 
            - logs.get("read:left_shoulder_pitch", t);
        error = pow(error, 2);
        sumError += error;
        if (maxError < 0.0 || maxError < error) {
            maxError = error;
            maxErrorTime = t;
        }
        count += 1.0;
        plot.add(Leph::VectorLabel(
            "t", t, 
            "read", logs.get("read:left_shoulder_pitch", t),
            "goal", logs.get("goal:left_shoulder_pitch", t),
            "simPos", sim.positions()(modelSim.getDOFIndex("left_shoulder_pitch")),
            //"simVel", sim.velocities()(modelSim.getDOFIndex("left_shoulder_pitch")),
            //"simAcc", sim.accelerations()(modelSim.getDOFIndex("left_shoulder_pitch")),
            //"backlash", sim.jointModel("left_shoulder_pitch").getBacklashState(),
            "error", error
        ));
        if (verbose) {
            Leph::ModelDraw(modelSim, *viewer);
            Leph::ModelDraw(modelRead, *viewer);
        }
    }
    
    cost = 0.2*(sumError/count) + 0.8*maxError;
    cost = sqrt(cost)*180.0/M_PI;
    
    if (verbose) {
        delete viewer;
        std::cout << "MeanError: " << sqrt(sumError/count)*180.0/M_PI << std::endl;
        std::cout << "MaxError:  " << sqrt(maxError)*180.0/M_PI << std::endl;
        std::cout << "MaxTime:   " << std::setprecision(10) << maxErrorTime << std::endl;
        std::cout << "Cost:      " << cost << std::endl;
        plot.plot("t", "all").render();
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
        "../../These/Data/logs-2016-10-05-arm-dynamics/arm_1",
        "../../These/Data/logs-2016-10-05-arm-dynamics/arm_2",
        "../../These/Data/logs-2016-10-05-arm-dynamics/arm_3",
        "../../These/Data/logs-2016-10-05-arm-dynamics/arm_4",
        "../../These/Data/logs-2016-10-05-arm-dynamics/arm_5",
    };

    //Initialize joint model parameters
    Leph::JointModel tmpJoint(Leph::JointModel::JointActuated, "tmp");
    Eigen::VectorXd initParams = tmpJoint.getParameters();
    sizeJointParameters = initParams.size();
    initParams <<
        0.000187767361335624,
        2.6280623085881,
        0.251943643687315,
        1.14155383637433,
        0.00237072388781183,
        12.278546662782,
        4.19281091164888,
        11.6048527958962,
        0.0339800463192264,
        0.01,
        0.1;
    initParams <<
        0.000180534242300923,
        3.27975358180819,
        0.231572584872825,
        1.34386985447662,
        0.00108776584363242,
        11.9949072324344,
        3.73391961447474,
        13.1822355976431,
        0.0335126612266252,
        0.00680511926306027,
        0.1383034099226;
    initParams <<
        0.00192576003116526,
        3.63823475370741,
        0.190634113069743,
        2.92521946266041,
        0.000352172621829293,
        17.3297300219461,
        5.64645976532205,
        12.3777471809961,
        0.0328840813138452,
        0.005,
        /*
        2.19408531779656,
        0.0170660846476906,
        0.0440792195453981,
        0.000410050308032774;
        */
        0.01;
    initParams <<
        0.00233879052181243,
        3.71669614845863,
        0.162539217146832,
        3.69063111598062,
        0.000408492253049055,
        21.060031997632,
        5.94705506228297,
        14.034335998561,
        0.0331105427578845,
        0.00362546489446558,
        0.0103287987739863;

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
        double initScore = scoreFitness(filenames[i], initParams, true);
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
        //Bound positive parameters
        Eigen::VectorXd tmpParams = params;
        double cost = 0.0;
        for (size_t i=0;i<(size_t)params.size();i++) {
            if (i < sizeJointParameters && tmpParams(i) < 0.0) {
                cost += 1000.0 - 1000.0*tmpParams(i);
                tmpParams(i) = 0.0;
            }
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
    libcmaes::CMAParameters<> cmaparams(initParams, 0.2, 10);
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

