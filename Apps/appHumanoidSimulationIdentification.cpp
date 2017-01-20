#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <libcmaes/cmaes.h>
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidSimulation.hpp"
#include "Model/JointModel.hpp"
#include "Types/MapSeries.hpp"
#include "Utils/Angle.h"
#include "Model/NamesModel.h"

#ifdef LEPH_VIEWER_ENABLED
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Plot/Plot.hpp"
#endif

/**
 * Names of frame whose inertia data is optimized
 */
static const std::vector<std::string> namesFrameInertia = {
    "trunk",
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
};

/**
 * Names of DOF whose Joint Model is uniquely optimized
 */
static const std::vector<std::string> namesDOFJoint = {
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
};

/**
 * Global CMA-ES configuration
 */
static const int cmaesElitismLevel = 1;
static const unsigned int cmaesMaxIterations = 10000;
static const unsigned int cmaesRestarts = 5;
static const unsigned int cmaesLambda = 10;
static const double cmaesSigma = -1.0;

/**
 * Global weighting ratio 
 * on trunk orientation errors
 */
static const double trunkOrientationCoef = 0.5;

/**
 * Simulate and compute the error between given logs
 * sequence and simulated model with given parameters.
 */
static double scoreFitness(
    const Leph::MapSeries& logs, 
    const Eigen::VectorXd& parameters, 
    size_t indexStartJoints,
    size_t indexStartInertias,
    size_t sizeJointParameters,
    size_t sizeInertiaParameters,
    const Eigen::MatrixXd& defaultInertiaData,
    const std::map<std::string, size_t>& defaultInertiaName,
    int verbose,
    double* sumError = nullptr, 
    double* countError = nullptr, 
    double* maxError = nullptr, 
    Eigen::VectorXd* maxAllError = nullptr)
{
    //Retrieve log time bounds
    double minTime = logs.timeMin();
    double maxTime = logs.timeMax();

    //Build used inertia data
    Eigen::MatrixXd currentInertiaData = defaultInertiaData;
    if (indexStartInertias != (size_t)-1) {
        size_t index = 0;
        for (const std::string& name : namesFrameInertia) {
            currentInertiaData.row(defaultInertiaName.at(name)) = 
                parameters.segment(
                    indexStartInertias + index*sizeInertiaParameters, sizeInertiaParameters)
                    .transpose();
            index++;
        }
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
    
    //Initialize full humanoid model
    //simulation with overrided 
    //inertia data
    Leph::HumanoidSimulation sim(
        Leph::SigmabanModel, 
        currentInertiaData, 
        defaultInertiaName);
    Leph::HumanoidModel modelRead(
        Leph::SigmabanModel, 
        "left_foot_tip", false);

    //Assign default joint parameters
    sim.setJointModelParameters(parameters.segment(0, sizeJointParameters));
    //Assign joint parameters to uniquely optimized DOF
    if (indexStartJoints != (size_t)-1) {
        size_t index = 0;
        for (const std::string& name : namesDOFJoint) {
            sim.jointModel(name).setParameters(
                parameters.segment(indexStartJoints + index*sizeJointParameters, sizeJointParameters));
            index++;
        }
    }

    //State initialization
    for (const std::string& name : Leph::NamesDOF) {
        //Initialization pos, vel and goal
        sim.setPos(name, logs.get("read:" + name, minTime));
        sim.setGoal(name, logs.get("read:" + name, minTime));
        sim.setVel(name, 0.0);
        //Reset backlash state
        sim.jointModel(name).resetBacklashState();
    }
    for (const std::string& name : Leph::NamesBase) {
        //Init base vel
        sim.setVel(name, 0.0); 
    }
    //Init model state
    sim.putOnGround(
        Leph::HumanoidFixedModel::LeftSupportFoot);
    sim.putFootAt(0.0, 0.0, 
        Leph::HumanoidFixedModel::LeftSupportFoot);
    //Run small time 0.5s for 
    //waiting stabilization (backlash)
    for (int k=0;k<500;k++) {
        sim.update(0.001);
    }
    
    //Main loop
    double tmpMax = -1.0;
    double tmpCount = 0.0;
    double tmpSum = 0.0;
    double tmpMaxTime = 0.0;
    Eigen::VectorXd tmpMaxAll(Leph::NamesDOF.size() + 2);
    for (size_t i=0;i<Leph::NamesDOF.size() + 2;i++) {
        tmpMaxAll(i) = -1.0;
    }
    std::string tmpMaxName = "";
#ifdef LEPH_VIEWER_ENABLED
    Leph::ModelViewer* viewer = nullptr;
    if (verbose >= 3) {
        viewer = new Leph::ModelViewer(1200, 900);
    }
    Leph::Plot plot;
#endif
    double incrStep = 0.01;
    int incrLoop = 10;
    for (double t=minTime;t<maxTime;t+=incrStep) {
#ifdef LEPH_VIEWER_ENABLED
        if (verbose >= 3) {
            if (!viewer->update()) {
                break;
            }
        }
#endif
        //Assign motor goal
        for (const std::string& name : Leph::NamesDOF) {
            sim.setGoal(name, logs.get("goal:" + name, t));
            modelRead.setDOF(name, logs.get("read:" + name, t));
        }
        //Run simulation
        for (int k=0;k<incrLoop;k++) {
            sim.update(0.001);
        }
        //Compute DOF error
        size_t index = 0;
        for (const std::string& name : Leph::NamesDOF) {
            double error = pow(
                180.0/M_PI*Leph::AngleDistance(sim.getPos(name), modelRead.getDOF(name)), 
                2);  
            tmpSum += error;
            tmpCount += 1.0;
            if (tmpMax < 0.0 || tmpMax < error) {
                tmpMax = error;
                tmpMaxTime = t-minTime;
                tmpMaxName = name;
            }
            if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < error) {
                tmpMaxAll(index) = error;
            }
            index++;
        }
        //Compute trunk orientation error
        Eigen::Vector3d simTrunkAngles = sim.model().trunkSelfOrientation();
        double errorTrunkRoll = pow(
            trunkOrientationCoef*180.0/M_PI*Leph::AngleDistance(
                simTrunkAngles.x(), logs.get("read:imu_roll", t)),
            2);
        double errorTrunkPitch = pow(
            trunkOrientationCoef*180.0/M_PI*Leph::AngleDistance(
                simTrunkAngles.y(), logs.get("read:imu_pitch", t)),
            2);
        tmpSum += errorTrunkRoll;
        tmpSum += errorTrunkPitch;
        tmpCount += 2.0;
        if (tmpMax < 0.0 || tmpMax < errorTrunkRoll) {
            tmpMax = errorTrunkRoll;
            tmpMaxTime = t-minTime;
            tmpMaxName = "trunk_roll";
        }
        if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < errorTrunkRoll) {
            tmpMaxAll(index) = errorTrunkRoll;
        }
        index++;
        if (tmpMax < 0.0 || tmpMax < errorTrunkPitch) {
            tmpMax = errorTrunkPitch;
            tmpMaxTime = t-minTime;
            tmpMaxName = "trunk_pitch";
        }
        if (tmpMaxAll(index) < 0.0 || tmpMaxAll(index) < errorTrunkPitch) {
            tmpMaxAll(index) = errorTrunkPitch;
        }
        index++;
        //Verbose Plot
#ifdef LEPH_VIEWER_ENABLED
        Leph::VectorLabel vect;
        if (verbose >= 2) {
            for (const std::string& name : Leph::NamesDOF) {
                vect.setOrAppend("t", t);
                vect.setOrAppend("read:" + name, 180.0/M_PI*logs.get("read:" + name, t));
                vect.setOrAppend("goal:" + name, 180.0/M_PI*logs.get("goal:" + name, t));
                vect.setOrAppend("sim:" + name, 180.0/M_PI*sim.getPos(name));
            }
            vect.setOrAppend("read:trunk_pitch", 180.0/M_PI*logs.get("read:imu_pitch", t));
            vect.setOrAppend("read:trunk_roll", 180.0/M_PI*logs.get("read:imu_roll", t));
            vect.setOrAppend("sim:trunk_roll", 180.0/M_PI*simTrunkAngles.x());
            vect.setOrAppend("sim:trunk_pitch", 180.0/M_PI*simTrunkAngles.y());
            plot.add(vect);
        }
        if (verbose >= 3) {
            Leph::CleatsDraw(sim, *viewer);
            Leph::ModelDraw(sim.model(), *viewer, 1.0);
            Leph::ModelDraw(modelRead, *viewer, 0.5);
        }
#endif
    }
    
    if (verbose >= 1) {
        std::cout << "MeanError: " << sqrt(tmpSum/tmpCount) << std::endl;
        std::cout << "MaxError: " << sqrt(tmpMax) << std::endl;
        std::cout << "MaxErrorTime: " << tmpMaxTime << std::endl;
        std::cout << "MaxErrorName: " << tmpMaxName << std::endl;
        std::cout << "MaxAllErrorMean: " << sqrt(tmpMaxAll.mean()) << std::endl;
        std::cout << "MaxAllError: " << (tmpMaxAll.array().sqrt().transpose()) << std::endl;
    }
#ifdef LEPH_VIEWER_ENABLED
    if (verbose >= 2) {
        plot
            .plot("t", "read:left_ankle_roll")
            .plot("t", "goal:left_ankle_roll")
            .plot("t", "sim:left_ankle_roll")
            .plot("t", "read:left_ankle_pitch")
            .plot("t", "goal:left_ankle_pitch")
            .plot("t", "sim:left_ankle_pitch")
            .plot("t", "read:left_knee")
            .plot("t", "goal:left_knee")
            .plot("t", "sim:left_knee")
            .plot("t", "read:left_hip_pitch")
            .plot("t", "goal:left_hip_pitch")
            .plot("t", "sim:left_hip_pitch")
            .plot("t", "read:left_hip_roll")
            .plot("t", "goal:left_hip_roll")
            .plot("t", "sim:left_hip_roll")
            .plot("t", "read:left_hip_yaw")
            .plot("t", "goal:left_hip_yaw")
            .plot("t", "sim:left_hip_yaw")
            .render();
        plot
            .plot("t", "read:right_ankle_roll")
            .plot("t", "goal:right_ankle_roll")
            .plot("t", "sim:right_ankle_roll")
            .plot("t", "read:right_ankle_pitch")
            .plot("t", "goal:right_ankle_pitch")
            .plot("t", "sim:right_ankle_pitch")
            .plot("t", "read:right_knee")
            .plot("t", "goal:right_knee")
            .plot("t", "sim:right_knee")
            .plot("t", "read:right_hip_pitch")
            .plot("t", "goal:right_hip_pitch")
            .plot("t", "sim:right_hip_pitch")
            .plot("t", "read:right_hip_roll")
            .plot("t", "goal:right_hip_roll")
            .plot("t", "sim:right_hip_roll")
            .plot("t", "read:right_hip_yaw")
            .plot("t", "goal:right_hip_yaw")
            .plot("t", "sim:right_hip_yaw")
            .render();
        plot
            .plot("t", "read:trunk_pitch")
            .plot("t", "read:trunk_roll")
            .plot("t", "sim:trunk_pitch")
            .plot("t", "sim:trunk_roll")
            .render();
    }
    if (verbose >= 3) {
        delete viewer;
    }
#endif
    if (sumError != nullptr) {
        *sumError += tmpSum;
    }
    if (countError != nullptr) {
        *countError += tmpCount;
    }
    if (maxError != nullptr && (*maxError < 0.0 || *maxError < tmpMax)) {
        *maxError = tmpMax;
    }
    if (maxAllError != nullptr) {
        for (size_t i=0;i<Leph::NamesDOF.size() + 2;i++) {
            if (tmpMaxAll(i) > maxAllError->operator()(i)) {
                maxAllError->operator()(i) = tmpMaxAll(i);
            }
        }
    }

    return 0.0;
}

/**
 * Use CMA-ES optimization to find
 * joint parameters and/or intertia
 * parameters for which forward simulation
 * to match logged motion
 */
int main(int argc, char** argv)
{
    //Parse user inputs
    if (
        argc < 3 || (
        std::string(argv[1]) != "SINGLE" &&
        std::string(argv[1]) != "JOINTS" &&
        std::string(argv[1]) != "INERTIAS" &&
        std::string(argv[1]) != "ALL"
        )
    ) {
        std::cout << "./app SINGLE   [logfile.mapseries] ..." << std::endl;
        std::cout << "./app JOINTS   [logfile.mapseries] ..." << std::endl;
        std::cout << "./app INERTIAS [logfile.mapseries] ..." << std::endl;
        std::cout << "./app ALL      [logfile.mapseries] ..." << std::endl;
        return 1;
    }
    //Retrieve log filenames
    std::vector<std::string> filenames;
    for (size_t i=2;i<(size_t)argc;i++) {
        filenames.push_back(std::string(argv[i]));
    }
    //Retrieve identification mode
    bool isIdentificationJoints = false;
    bool isIdentificationInertias = false;
    if (std::string(argv[1]) == "SINGLE") {
        isIdentificationJoints = false;
        isIdentificationInertias = false;
    } else if (std::string(argv[1]) == "JOINTS") {
        isIdentificationJoints = true;
        isIdentificationInertias = false;
    } else if (std::string(argv[1]) == "INERTIAS") {
        isIdentificationJoints = false;
        isIdentificationInertias = true;
    } else if (std::string(argv[1]) == "ALL") {
        isIdentificationJoints = true;
        isIdentificationInertias = true;
    } else {
        return 1;
    }

    //Load data logs into MapSeries
    std::vector<Leph::MapSeries> logsData;
    for (size_t i=0;i<filenames.size();i++) {
        logsData.push_back(Leph::MapSeries());
        logsData.back().importData(filenames[i]);
        std::cout << "Loaded " 
            << filenames[i] << ": "
            << logsData.back().dimension() << " series from " 
            << logsData.back().timeMin() << "s to " 
            << logsData.back().timeMax() << "s with length "
            << logsData.back().timeMax()-logsData.back().timeMin() 
            << "s" << std::endl;
    }

    //Inertia default data and name
    Eigen::MatrixXd defaultInertiaData;
    std::map<std::string, size_t> defaultInertiaName;
    //Load default inertia data from model
    Leph::HumanoidModel tmpModel(
        Leph::SigmabanModel, "left_foot_tip", false);
    defaultInertiaData = tmpModel.getInertiaData();
    defaultInertiaName = tmpModel.getInertiaName();

    //Joint default parameters
    Eigen::VectorXd defaultJointParams;
    //Load default joint model parameters
    Leph::JointModel tmpJoint;
    defaultJointParams = tmpJoint.getParameters();

    //Build initial parameters vector
    Eigen::VectorXd initParams = defaultJointParams;
    size_t indexStartJoints = (size_t)-1;
    size_t indexStartInertias = (size_t)-1;
    //Util sizes
    size_t sizeInertiaParameters = defaultInertiaData.cols();
    size_t sizeJointParameters = defaultJointParams.size();
    size_t sizeAllParameters = sizeJointParameters;
    //Add all uniquely optimized joint parameters
    if (isIdentificationJoints) {
        indexStartJoints = sizeAllParameters;
        sizeAllParameters += namesDOFJoint.size()*sizeJointParameters;
        initParams.conservativeResize(sizeAllParameters);
        size_t index = 0;
        for (const std::string& name : namesDOFJoint) {
            initParams.segment(
                indexStartJoints + index*sizeJointParameters, sizeJointParameters) = 
                defaultJointParams;
            index++;
        }
    }
    //Add all uniquely optimized inertia parameters
    if (isIdentificationInertias) {
        indexStartInertias = sizeAllParameters;
        sizeAllParameters += namesFrameInertia.size()*sizeInertiaParameters;
        initParams.conservativeResize(sizeAllParameters);
        size_t index = 0;
        for (const std::string& name : namesFrameInertia) {
            initParams.segment(
                indexStartInertias + index*sizeInertiaParameters, sizeInertiaParameters) = 
                defaultInertiaData.row(defaultInertiaName.at(name)).transpose();
            index++;
        }
    }

    //Normalization coefficient
    Eigen::VectorXd coef = initParams;
    Eigen::VectorXd coefInv = initParams;
    for (size_t i=0;i<(size_t)coef.size();i++) {
        if (fabs(coef(i)) < 1e-4) {
            coef(i) = 0.01;
        }
        coefInv(i) = 1.0/coef(i);
    }

    //Initial verbose
    for (size_t i=0;i<logsData.size();i++) {
        scoreFitness(
            logsData[i], initParams, 
            indexStartJoints, indexStartInertias, 
            sizeJointParameters, sizeInertiaParameters, 
            defaultInertiaData, defaultInertiaName, 3);
    }

    //Normalization of initial parameters
    initParams = coefInv.array() * initParams.array();
    Eigen::VectorXd bestParams = initParams;
    double bestScore = -1.0;
    int iteration = 1;
    
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&logsData, &coef, &indexStartJoints, &indexStartInertias, 
        &sizeJointParameters, &sizeInertiaParameters, 
        &defaultInertiaData, &defaultInertiaName]
        (const Eigen::VectorXd& params) 
    {
        //Check positive joint parameters
        double cost = 0.0;
        for (size_t i=0;i<(size_t)params.size();i++) {
            if (
                (indexStartInertias == (size_t)-1 || 
                i < indexStartInertias) && 
                params(i) < 0.0
            ) {
                cost += 1000.0 - 1000.0*params(i);
            }
        }
        if (cost > 0.0) {
            return cost;
        }
        //Iterate over on all logs
        double sumError = 0.0;
        double countError = 0.0;
        double maxError = -1.0;
        Eigen::VectorXd maxAllError(Leph::NamesDOF.size() + 2);
        for (size_t i=0;i<Leph::NamesDOF.size() + 2;i++) {
            maxAllError(i) = -1.0;
        }
        for (size_t i=0;i<logsData.size();i++) {
            try {
                cost += scoreFitness(
                    logsData[i], coef.array() * params.array(),
                    indexStartJoints, indexStartInertias, 
                    sizeJointParameters, sizeInertiaParameters, 
                    defaultInertiaData, defaultInertiaName, 0,
                    &sumError, &countError, &maxError, &maxAllError);
            } catch (const std::runtime_error& e) {
                cost += 10000.0;
            }
        }
        if (countError > 0.0 && maxError > 0.0) {
            return 
                cost 
                + 0.6*(sumError/countError) 
                + 0.2*(maxAllError.mean()) 
                + 0.2*(maxError);
        } else {
            return cost;
        }
    };
    
    //Progress function
    libcmaes::ProgressFunc<
        libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress = 
        [&bestParams, &bestScore, &iteration, &coef, &logsData,
        &indexStartJoints, &indexStartInertias, 
        &sizeJointParameters, &sizeInertiaParameters, 
        &defaultInertiaData, &defaultInertiaName]
        (const libcmaes::CMAParameters<>& cmaparams, 
        const libcmaes::CMASolutions& cmasols)
    {
        //Retrieve best Trajectories and score
        Eigen::VectorXd params = coef.array() * 
            cmasols.get_best_seen_candidate().get_x_dvec().array();
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
            std::cout << "Params: ";
            for (size_t i=0;i<(size_t)params.size();i++) {
                std::cout << std::setprecision(10) << params(i);
                if (i != (size_t)params.size()-1) {
                    std::cout << ", ";
                } else {
                    std::cout << ";" << std::endl;
                }
            }
            std::cout << "============" << std::endl;
            for (size_t i=0;i<logsData.size();i++) {
                scoreFitness(
                    logsData[i], bestParams, 
                    indexStartJoints, indexStartInertias, 
                    sizeJointParameters, sizeInertiaParameters, 
                    defaultInertiaData, defaultInertiaName, 1);
            }
            std::cout << "============" << std::endl;
        }
        iteration++;
        
        //Call default CMA-ES default progress function
	return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
            ::_defaultPFunc(cmaparams, cmasols);
    };
    
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(initParams, 
        cmaesSigma, cmaesLambda);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(cmaesElitismLevel);
    cmaparams.set_restarts(cmaesRestarts);
    cmaparams.set_max_iter(cmaesMaxIterations);
    cmaparams.set_ftolerance(1e-9);
    
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams, progress);
    
    //Retrieve best Trajectories and score
    bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
    bestScore = cmasols.get_best_seen_candidate().get_fvalue();
    
    //Final verbose
    for (size_t i=0;i<logsData.size();i++) {
        scoreFitness(
            logsData[i], bestParams.array(), 
            indexStartJoints, indexStartInertias, 
            sizeJointParameters, sizeInertiaParameters, 
            defaultInertiaData, defaultInertiaName, 3);
    }
    std::cout << "BestParams: " << bestParams.transpose() << std::endl;
    std::cout << "BestScore: " << bestScore << std::endl;

    return 0;
}

