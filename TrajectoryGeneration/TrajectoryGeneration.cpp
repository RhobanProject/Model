#include <libcmaes/cmaes.h>
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "Utils/FileEigen.h"
#include "Utils/FileModelParameters.h"
#include "Utils/time.h"
#include "Model/NamesModel.h"

namespace Leph {

TrajectoryGeneration::TrajectoryGeneration(RobotType type,
    const std::string& modelParamsPath) :
    _type(type),
    _modelParametersPath(modelParamsPath),
    _initialParameters(),
    _normCoefs(),
    _generateFunc(),
    _checkParamsFunc(),
    _checkStateFunc(),
    _checkDOFFunc(),
    _scoreFunc(),
    _endScoreFunc(),
    _bestTraj(),
    _bestParams(),
    _bestScore(0.0),
    _countIteration(1)
{
}
        
void TrajectoryGeneration::setInitialParameters(
    const Eigen::VectorXd& params)
{
    _initialParameters = params;
}

void TrajectoryGeneration::setNormalizationCoefs(
    const Eigen::VectorXd& normCoefs)
{
    if (normCoefs.size() != _initialParameters.size()) {
        throw std::logic_error(
            "TrajectoryGeneration invalid norm coefs size: "
            + std::to_string(normCoefs.size()));
    }
    _normCoefs = normCoefs;
}
 
void TrajectoryGeneration::setTrajectoryGenerationFunc(
    GenerationFunc func)
{
    _generateFunc = func;
}
        
void TrajectoryGeneration::setCheckParametersFunc(
    CheckParamsFunc func)
{
    _checkParamsFunc = func;
}
        
void TrajectoryGeneration::setCheckStateFunc(
    CheckStateFunc func)
{
    _checkStateFunc = func;
}
        
void TrajectoryGeneration::setCheckDOFFunc(
    CheckDOFFunc func)
{
    _checkDOFFunc = func;
}
        
void TrajectoryGeneration::setScoreFunc(
    ScoreFunc func)
{
    _scoreFunc = func;
}
        
void TrajectoryGeneration::setEndScoreFunc(
    EndScoreFunc func)
{
    _endScoreFunc = func;
}
        
Eigen::VectorXd TrajectoryGeneration::initialParameters() const
{
    return _initialParameters;
}

Eigen::VectorXd TrajectoryGeneration::normalizationCoefs() const
{
    if (_normCoefs.size() == 0) {
        return Eigen::VectorXd::Ones(_initialParameters.size());
    } else {
        return _normCoefs;
    }
}        
        
Trajectories TrajectoryGeneration::generateTrajectory(
    const Eigen::VectorXd& params) const
{
    return _generateFunc(params);
}
        
double TrajectoryGeneration::checkParameters(
    const Eigen::VectorXd& params) const
{
    return _checkParamsFunc(params);
}
double TrajectoryGeneration::checkState(
    const Eigen::VectorXd& params,
    double t,
    const Eigen::Vector3d& trunkPos,
    const Eigen::Vector3d& trunkAxis,
    const Eigen::Vector3d& footPos,
    const Eigen::Vector3d& footAxis) const
{
    return _checkStateFunc(params, t, 
        trunkPos, trunkAxis, footPos, footAxis);
}
double TrajectoryGeneration::checkDOF(
    const Eigen::VectorXd& params,
    double t,
    const HumanoidFixedModel& model) const
{
    return _checkDOFFunc(params, t, model);
}
        
double TrajectoryGeneration::score(
    double t,
    HumanoidFixedModel& model,
    const std::map<std::string, JointModel>& joints,
    const Eigen::VectorXd& torques,
    const Eigen::VectorXd& dq,
    const Eigen::VectorXd& ddq,
    bool isDoubleSupport,
    HumanoidFixedModel::SupportFoot supportFoot,
    std::vector<double>& data) const
{
    return _scoreFunc(
        t, model, joints,
        torques, dq, ddq, 
        isDoubleSupport, supportFoot, 
        data);
}
double TrajectoryGeneration::endScore(
    const Eigen::VectorXd& params,
    const Trajectories& traj,
    double score,
    std::vector<double>& data,
    bool verbose) const
{
    return _endScoreFunc(params, traj, 
        score, data, verbose);
}
        
double TrajectoryGeneration::scoreTrajectory(
    const Eigen::VectorXd& params,
    bool verbose) const
{
    double cost = checkParameters(params);
    if (cost > 0.0) {
        if (verbose) {
            std::cout 
                << "Error checkParameters() cost=" 
                << cost << std::endl;
        }
        return cost;
    } else {
        Trajectories traj = generateTrajectory(params);
        return scoreTrajectory(params, traj, verbose);
    }
}
double TrajectoryGeneration::scoreTrajectory(
    const Eigen::VectorXd& params,
    const Trajectories& traj,
    bool verbose) const
{
    //Load model parameters
    Eigen::MatrixXd jointData;
    std::map<std::string, size_t> jointName;
    Eigen::MatrixXd inertiaData;
    std::map<std::string, size_t> inertiaName;
    Eigen::MatrixXd geometryData;
    std::map<std::string, size_t> geometryName;
    if (_modelParametersPath != "") {
        ReadModelParameters(
            _modelParametersPath,
            jointData, jointName,
            inertiaData, inertiaName,
            geometryData, geometryName);
    }
    //Joint Model for each DOF
    std::map<std::string, JointModel> joints;
    for (const std::string& name : NamesDOF) {
        joints[name] = JointModel();
        if (
            _modelParametersPath != "" && 
            jointName.count(name) > 0
        ) {
            joints[name].setParameters(
                jointData.row(jointName.at(name)).transpose());
        } 
    }
    //Sigmaban fixed model
    Leph::HumanoidFixedModel model(_type, 
        inertiaData, inertiaName, 
        geometryData, geometryName);
    double cost = 0.0;
    std::vector<double> data;
    for (double t=traj.min();t<=traj.max();t+=0.01) {
        //Check Cartesian State
        Eigen::Vector3d trunkPos;
        Eigen::Vector3d trunkAxis;
        Eigen::Vector3d footPos;
        Eigen::Vector3d footAxis;
        TrajectoriesTrunkFootPos(t, traj,
            trunkPos, trunkAxis,
            footPos, footAxis);
        double costState = checkState(params, t, 
            trunkPos, trunkAxis, footPos, footAxis);
        if (costState > 0.0) {
            if (verbose) {
                std::cout 
                    << "Error checkState() cost=" 
                    << 1000.0 + costState 
                    << std::endl;
            }
            cost += 1000.0 + costState;
            continue;
        }
        //Compute kinematics
        Eigen::VectorXd dq;
        Eigen::VectorXd ddq;
        double boundIKDistance = 0.0;
        bool isIKSuccess = TrajectoriesComputeKinematics(
            t, traj, model, dq, ddq, &boundIKDistance);
        //Cost near IK bound
        double boundIKThreshold = 1e-2;
        if (boundIKDistance < boundIKThreshold) {
            if (verbose) {
                std::cout 
                    << "Warning boundIKDistance=" 
                    << boundIKDistance << std::endl;
            }
            cost += 1000.0 
                + 1000.0*(boundIKThreshold - boundIKDistance);
        }
        if (!isIKSuccess) {
            if (verbose) {
                std::cout 
                    << "Error checkIK() cost=" 
                    << 1000.0 << std::endl;
            }
            cost += 1000.0;
            continue;
        }
        //Check Joit DOF
        double costDOF = checkDOF(params, t, model);
        if (costDOF > 0.0) {
            if (verbose) {
                std::cout 
                    << "Error checkDOF() cost=" 
                    << 1000.0 + costDOF 
                    << std::endl;
            }
            cost += 1000.0 + costDOF;
            continue;
        }
        //Compute DOF torques
        bool isDoubleSupport;
        HumanoidFixedModel::SupportFoot supportFoot;
        TrajectoriesSupportFootState(t, traj,
            isDoubleSupport, supportFoot);
        Eigen::VectorXd torques;
        if (isDoubleSupport) {
            if (supportFoot == HumanoidFixedModel::LeftSupportFoot) {
                torques = model.get().inverseDynamicsClosedLoop(
                    "right_foot_tip", false, dq, ddq);
            } else {
                torques = model.get().inverseDynamicsClosedLoop(
                    "left_foot_tip", false, dq, ddq);
            }
        } else {
            torques = model.get().inverseDynamics(dq, ddq);
        }
        //Evaluate the trajectory
        cost += score(
            t, model, joints,
            torques, dq, ddq, 
            isDoubleSupport, supportFoot,
            data);
    }
    //Ending trajectory scoring
    cost += endScore(params, traj, cost, data, verbose);

    return cost;
}
        
void TrajectoryGeneration::runOptimization(
    unsigned int maxIterations,
    unsigned int restart,
    const std::string& filename,
    unsigned int populationSize,
    double lambda,
    unsigned int elitismLevel)
{
    //Retrieve initial parameters
    Eigen::VectorXd initParams = initialParameters();
    //Retrieve normalization coefficients
    const Eigen::VectorXd normCoef = normalizationCoefs();
    //Initialization
    _countIteration = 1;
    _bestScore = -1.0;

    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [this, &normCoef](const Eigen::VectorXd& params) 
    {
        return this->scoreTrajectory(
            params.array() * normCoef.array());
    };
    //Progress function
    libcmaes::ProgressFunc<
        libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress = 
        [this, &filename, &normCoef](const libcmaes::CMAParameters<>& cmaparams, 
            const libcmaes::CMASolutions& cmasols)
    {
        //Retrieve best Trajectories and score
        Eigen::VectorXd params = 
            cmasols.get_best_seen_candidate().get_x_dvec().array()
            * normCoef.array();
        double score = 
            cmasols.get_best_seen_candidate().get_fvalue();
        if (_bestScore < 0.0 || _bestScore > score) {
            _bestParams = params;
            _bestTraj = generateTrajectory(params);
            _bestScore = score;
        }
        //Save current best found
        if (_countIteration % 100 == 0) {
            std::cout << "============" 
                << std::endl;
            std::cout << "****** Date: " << currentDate() 
                << std::endl;
            if (filename != "") {
                _bestTraj.exportData(filename + ".splines");
                std::cout << "****** Saving Trajectories to: " 
                    << filename + ".splines" << std::endl;
                WriteEigenVector(filename + ".params", _bestParams);
                std::cout << "****** Saving Parameters to: " 
                    << filename + ".params" << std::endl;
            }
            std::cout << "****** Dimension: " 
                << _bestParams.size() << std::endl;
            std::cout << "****** BestScore: " 
                << _bestScore << std::endl;
            std::cout << "****** BestParams: " 
                << _bestParams.transpose() << std::endl;
            std::cout << "****** BestFitness verbose:" << std::endl;
            scoreTrajectory(_bestParams, true);
            std::cout << "****** CurrentScore: " 
                << score << std::endl;
            std::cout << "****** CurrentFitness verbose:" << std::endl;
            scoreTrajectory(params, true);
            std::cout << "============" 
                << std::endl;
        }
        _countIteration++;

        //Call default CMA-ES default progress function
	return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
            ::_defaultPFunc(cmaparams, cmasols);
    };

    //Show initial info
    double initScore = scoreTrajectory(initParams, true);
    std::cout << "============" << std::endl;
    std::cout << "Init Score: " << initScore << std::endl;
    std::cout << "Dimension:  " << initParams.size() << std::endl;
    std::cout << "============" << std::endl;

    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(
        initParams.array() / normCoef.array(), 
        lambda, populationSize);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(elitismLevel);
    cmaparams.set_restarts(restart);
    cmaparams.set_max_iter(maxIterations);

    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams, progress);

    //Retrieve best Trajectories and score
    Eigen::VectorXd params = 
        cmasols.get_best_seen_candidate().get_x_dvec().array()
        * normCoef.array();
    double score = 
        cmasols.get_best_seen_candidate().get_fvalue();
    _bestParams = params;
    _bestTraj = generateTrajectory(params);
    _bestScore = score;
    std::cout << "############" 
        << std::endl;
    if (filename != "") {
        _bestTraj.exportData(filename + ".splines");
        std::cout << "****** Saving Trajectories to: " 
            << filename + ".splines" << std::endl;
        WriteEigenVector(filename + ".params", _bestParams);
        std::cout << "****** Saving Parameters to: " 
            << filename + ".params" << std::endl;
    }
    std::cout << "****** BestScore: " 
        << _bestScore << std::endl;
    std::cout << "****** BestParams: " 
        << _bestParams.transpose() << std::endl;
    std::cout << "############" 
        << std::endl;
}
        
const Trajectories& TrajectoryGeneration::bestTrajectories() const
{
    return _bestTraj;
}
const Eigen::VectorXd& TrajectoryGeneration::bestParameters() const
{
    return _bestParams;
}
double TrajectoryGeneration::bestScore() const
{
    return _bestScore;
}

}

