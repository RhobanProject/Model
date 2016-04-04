#include <libcmaes/cmaes.h>
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"

namespace Leph {

TrajectoryGeneration::TrajectoryGeneration(RobotType type) :
    _type(type),
    _initialParameters(),
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
    const Eigen::Vector3d& trunkPos,
    const Eigen::Vector3d& trunkAxis,
    const Eigen::Vector3d& footPos,
    const Eigen::Vector3d& footAxis) const
{
    return _checkStateFunc(
        trunkPos, trunkAxis, footPos, footAxis);
}
double TrajectoryGeneration::checkDOF(
    const HumanoidFixedModel& model) const
{
    return _checkDOFFunc(model);
}
        
double TrajectoryGeneration::score(
    double t,
    HumanoidFixedModel& model,
    const Eigen::VectorXd& torques,
    const Eigen::VectorXd& dq,
    const Eigen::VectorXd& ddq,
    bool isDoubleSupport,
    HumanoidFixedModel::SupportFoot supportFoot,
    std::vector<double>& data) const
{
    return _scoreFunc(
        t, model, 
        torques, dq, ddq, 
        isDoubleSupport, supportFoot, 
        data);
}
double TrajectoryGeneration::endScore(
    const Eigen::VectorXd& params,
    const Trajectories& traj,
    std::vector<double>& data) const
{
    return _endScoreFunc(params, traj, data);
}
        
double TrajectoryGeneration::scoreTrajectory(
    const Eigen::VectorXd& params) const
{
    double cost = checkParameters(params);
    if (cost > 0.0) {
        return cost;
    } else {
        Trajectories traj = generateTrajectory(params);
        return scoreTrajectory(params, traj);
    }
}
double TrajectoryGeneration::scoreTrajectory(
    const Eigen::VectorXd& params,
    const Trajectories& traj) const
{
    //Sigmaban fixed model
    Leph::HumanoidFixedModel model(_type);
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
        double costState = checkState(
            trunkPos, trunkAxis, footPos, footAxis);
        if (costState > 0.0) {
            return 1000.0 + costState;
        }
        //Compute kinematics
        Eigen::VectorXd dq;
        Eigen::VectorXd ddq;
        bool isIKSuccess = TrajectoriesComputeKinematics(
            t, traj, model, dq, ddq);
        if (!isIKSuccess) {
            return 1000.0 + 1000.0*(traj.max()-t);
        }
        //Check Joit DOF
        double costDOF = checkDOF(model);
        if (costDOF > 0.0) {
            return 1000.0 + costDOF;
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
            t, model, 
            torques, dq, ddq, 
            isDoubleSupport, supportFoot,
            data);
    }
    //Ending trajectory scoring
    cost += endScore(params, traj, data);

    return cost;
}
        
void TrajectoryGeneration::runOptimization(
    unsigned int maxIterations,
    unsigned int restart,
    const std::string& filename,
    unsigned int populationSize,
    double lambda)
{
    //Initialization
    _countIteration = 1;
    _bestScore = -1.0;
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [this](const Eigen::VectorXd& params) 
    {
        return this->scoreTrajectory(params);
    };
    //Progress function
    libcmaes::ProgressFunc<
        libcmaes::CMAParameters<>, libcmaes::CMASolutions> progress = 
        [this, &filename](const libcmaes::CMAParameters<>& cmaparams, 
            const libcmaes::CMASolutions& cmasols)
    {
        //Retrieve best Trajectories and score
        Eigen::VectorXd params = 
            cmasols.get_best_seen_candidate().get_x_dvec();
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
            if (filename != "") {
                _bestTraj.exportData(filename);
                std::cout << "Saving Trajectories to: " 
                    << filename << std::endl;
            }
            std::cout << "BestScore: " 
                << _bestScore << std::endl;
            std::cout << "BestParams: " 
                << _bestParams.transpose() << std::endl;
            std::cout << "============" 
                << std::endl;
        }
        _countIteration++;

        //Call default CMA-ES default progress function
	return libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>
            ::_defaultPFunc(cmaparams, cmasols);
    };


    Eigen::VectorXd initParams = initialParameters();

    double initScore = scoreTrajectory(initParams);
    std::cout << "============" << std::endl;
    std::cout << "Init Score: " << initScore << std::endl;
    std::cout << "============" << std::endl;

    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(
        initParams, lambda, populationSize);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(true);
    cmaparams.set_restarts(restart);
    cmaparams.set_max_iter(maxIterations);

    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams, progress);
    //Retrieve best Trajectories and score
    Eigen::VectorXd params = 
        cmasols.get_best_seen_candidate().get_x_dvec();
    double score = 
        cmasols.get_best_seen_candidate().get_fvalue();
    _bestParams = params;
    _bestTraj = generateTrajectory(params);
    _bestScore = score;
    std::cout << "############" 
        << std::endl;
    if (filename != "") {
        _bestTraj.exportData(filename);
        std::cout << "Saving Trajectories to: " 
            << filename << std::endl;
    }
    std::cout << "BestScore: " 
        << _bestScore << std::endl;
    std::cout << "BestParams: " 
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

