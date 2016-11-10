#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <chrono>
#include <Eigen/Dense>
#include <stdexcept>
#include <libcmaes/cmaes.h>
#include "Plot/Plot.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/OdometryModel.hpp"
#include "Utils/Angle.h"

/**
 * Global configuration
 */
static const unsigned int CMAESMaxIteration = 5000;
static const unsigned int CMAESLambda = 20;
static const unsigned int CMAESRestart = 4;
static const bool CMAESQuiet = true;
static const bool CMAESElitism = true;
static const bool CMAESThreading = true;
static const unsigned int NumberTries = 10;
static const double AngularRangeFitness = 10.0*M_PI/180.0;
static const bool IsPrintCSV = true;
static const bool IsDebug = false;

/**
 * Global random generator
 */
static std::default_random_engine generator(
    std::chrono::system_clock::now().time_since_epoch().count());

/**
 * Odometry log data structure
 */
struct OdometryData {
    std::vector<std::vector<Eigen::Vector3d>> readTrajsPose;
    std::vector<std::vector<Leph::HumanoidFixedModel::SupportFoot>> readTrajsSupport;
    std::vector<std::vector<Eigen::Vector3d>> goalTrajsPose;
    std::vector<std::vector<Leph::HumanoidFixedModel::SupportFoot>> goalTrajsSupport;
    std::vector<std::vector<Eigen::Vector4d>> walkTrajsOrder;
    std::vector<std::vector<double>> walkTrajsPhase;
    std::vector<Eigen::Vector2d> targetDisplacements;
};

/**
 * Odometry computation type
 */
enum OdometryType {
    //No model. Use walk order as input
    OdometryOrder,
    //Use goal model state as input
    OdometryGoal,
    //Use read model state as input
    OdometryRead,
};

/**
 * Append data loaded from fiven filename to
 * the OdometryData structure
 */
void loadDataFromFile(OdometryData& data, const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error(
            "Unable to open log file: " + filename);
    }
    size_t lastSeq = -1;
    while (file.good() && file.peek() != EOF) {
        while (file.peek() == ' ' || file.peek() == '\n') {
            file.ignore();
        }
        size_t seq;
        size_t index;
        double readPoseX;
        double readPoseY;
        double readPoseYaw;
        int readSupportFoot;
        double goalPoseX;
        double goalPoseY;
        double goalPoseYaw;
        int goalSupportFoot;
        double walkOrderX;
        double walkOrderY;
        double walkOrderTheta;
        double walkOrderEnabled;
        double walkPhase;
        double targetX;
        double targetY;
        file >> seq;
        file >> index;
        file >> readPoseX;
        file >> readPoseY;
        file >> readPoseYaw;
        file >> readSupportFoot;
        file >> goalPoseX;
        file >> goalPoseY;
        file >> goalPoseYaw;
        file >> goalSupportFoot;
        file >> walkOrderX;
        file >> walkOrderY;
        file >> walkOrderTheta;
        file >> walkOrderEnabled;
        file >> walkPhase;
        file >> targetX;
        file >> targetY;
        if (lastSeq != seq) {
            data.readTrajsPose.push_back(std::vector<Eigen::Vector3d>());
            data.readTrajsSupport.push_back(std::vector<Leph::HumanoidFixedModel::SupportFoot>());
            data.goalTrajsPose.push_back(std::vector<Eigen::Vector3d>());
            data.goalTrajsSupport.push_back(std::vector<Leph::HumanoidFixedModel::SupportFoot>());
            data.walkTrajsOrder.push_back(std::vector<Eigen::Vector4d>());
            data.walkTrajsPhase.push_back(std::vector<double>());
            data.targetDisplacements.push_back(Eigen::Vector2d(targetX, targetY));
        }
        lastSeq = seq;
        data.readTrajsPose.back().push_back(Eigen::Vector3d(readPoseX, readPoseY, readPoseYaw));
        data.readTrajsSupport.back().push_back((Leph::HumanoidFixedModel::SupportFoot)readSupportFoot);
        data.goalTrajsPose.back().push_back(Eigen::Vector3d(goalPoseX, goalPoseY, goalPoseYaw));
        data.goalTrajsSupport.back().push_back((Leph::HumanoidFixedModel::SupportFoot)goalSupportFoot);
        data.walkTrajsOrder.back().push_back(Eigen::Vector4d(walkOrderX, walkOrderY, walkOrderTheta, walkOrderEnabled));
        data.walkTrajsPhase.back().push_back(walkPhase);
    }
    file.close();
}

/**
 * Shuffle given odometry data 
 */
void shuffleData(OdometryData& data)
{
    //Create the vector of sorted indexes
    size_t size = data.readTrajsPose.size();
    std::vector<size_t> indexes;
    for (size_t i=0;i<size;i++) {
        indexes.push_back(i);
    }
    //Shuffle the indexes vector
    std::shuffle(indexes.begin(), indexes.end(), generator);

    //Copy last data
    OdometryData lastData = data;
    //Do shuffle
    for (size_t i=0;i<size;i++) {
        size_t index = indexes[i];
        data.readTrajsPose[i] = lastData.readTrajsPose[index];
        data.readTrajsSupport[i] = lastData.readTrajsSupport[index];
        data.goalTrajsPose[i] = lastData.goalTrajsPose[index];
        data.goalTrajsSupport[i] = lastData.goalTrajsSupport[index];
        data.walkTrajsOrder[i] = lastData.walkTrajsOrder[index];
        data.walkTrajsPhase[i] = lastData.walkTrajsPhase[index];
        data.targetDisplacements[i] = lastData.targetDisplacements[index];
    }
}

/**
 * Display given data sequence and computed odometry
 */
void displaySequence(const OdometryData& data, size_t index)
{
    if (index >= data.readTrajsPose.size()) {
        throw std::logic_error("Invalid index");
    }
    Leph::Plot plot;
    for (size_t i=0;i<data.readTrajsPose[index].size();i++) {
        plot.add(Leph::VectorLabel(
            "order_x", data.walkTrajsOrder[index][i].x(),
            "order_y", data.walkTrajsOrder[index][i].y(),
            "order_theta", data.walkTrajsOrder[index][i].z(),
            "walk_phase", data.walkTrajsPhase[index][i],
            "read_x", data.readTrajsPose[index][i].x(),
            "read_y", data.readTrajsPose[index][i].y(),
            "goal_x", data.goalTrajsPose[index][i].x(),
            "goal_y", data.goalTrajsPose[index][i].y()
        ));
    }
    plot.plot("index", "all").render();
}

double distanceFromArc(double x, double y, double targetX, double targetY, double angularRange)
{
    //Convert target in polar coordinate
    double targetRadius = sqrt(targetX*targetX + targetY*targetY);
    double targetAngle = atan2(targetY, targetX);
    //Compute cartesian position of allowed arc
    double targetX1 = targetRadius*cos(targetAngle+angularRange);
    double targetY1 = targetRadius*sin(targetAngle+angularRange);
    double targetX2 = targetRadius*cos(targetAngle-angularRange);
    double targetY2 = targetRadius*sin(targetAngle-angularRange);
    //Compute distance from point to arc, and external arc target points
    double radius = sqrt(x*x + y*y);
    double angle = atan2(y, x);
    double dist1 = fabs(targetRadius - radius);
    double dist2 = sqrt(pow(targetX1-x, 2) + pow(targetY1-y, 2));
    double dist3 = sqrt(pow(targetX2-x, 2) + pow(targetY2-y, 2));
    //Return the minimum distance
    if (dist1 <= dist2 && dist1 <= dist3 && fabs(Leph::AngleDistance(angle, targetAngle)) <= angularRange) {
        return dist1;
    } else if (dist2 <= dist3) {
        return dist2;
    } else {
        return dist3;
    }
}

/**
 * Evaluate and return for given odometry type, model and parameters
 * the position RMSE using sequences between start and end indexes
 */
double odometryModelFitness(
    const OdometryData& data, OdometryType type, 
    Leph::OdometryModel::OdometryModelType model, 
    const Eigen::VectorXd parameters, 
    size_t indexStart, size_t indexEnd,
    bool isLearningScore,
    Leph::Plot* plot = nullptr)
{
    double error = 0.0;
    int count = 0;
    //Iterate over given sequences
    for (size_t i=indexStart;i<=indexEnd;i++) {
        //Assign odometry model parameters
        Leph::OdometryModel odometry(model);
        odometry.parameters() = parameters;
        odometry.reset();
        //Plot model
        Leph::OdometryModel* odometryDebugRead = nullptr;
        Leph::OdometryModel* odometryDebugGoal = nullptr;
        Leph::OdometryModel* odometryDebugOrder = nullptr;
        if (plot != nullptr) {
            odometryDebugRead = new Leph::OdometryModel(model);
            odometryDebugRead->reset();
            odometryDebugGoal = new Leph::OdometryModel(model);
            odometryDebugGoal->reset();
            odometryDebugOrder = new Leph::OdometryModel(model);
            odometryDebugOrder->reset();
            plot->add(Leph::VectorLabel(
                "target_x", 0.0,
                "target_y", 0.0
            ));
        }
        //Check parameter bounds
        Eigen::VectorXd lowerBounds = odometry.parameterLowerBounds();
        Eigen::VectorXd upperBounds = odometry.parameterUpperBounds();
        for (size_t j=0;j<(size_t)parameters.size();j++) {
            if (parameters(j) < lowerBounds(j)) {
                error += 1000.0 + 1000.0*(lowerBounds(j) - parameters(j));
            }
            if (parameters(j) > upperBounds(j)) {
                error += 1000.0 + 1000.0*(parameters(j) - upperBounds(j));
            }
        }
        //Oterate over recorded point inside a sequence
        double lastPhase = data.walkTrajsPhase[i].front();
        for (size_t j=0;j<data.readTrajsPose[i].size();j++) {
            double phase = data.walkTrajsPhase[i][j];
            //Use given odometry type
            if (type == OdometryOrder) {
                if (lastPhase > 0.8 && phase < 0.2) {
                    double enableGain = data.walkTrajsOrder[i][j](3);
                    odometry.updateFullStep(
                        2.0*data.walkTrajsOrder[i][j].segment(0, 3)
                        *enableGain);
                }
            } else if (type == OdometryGoal) {
                odometry.update(
                    data.goalTrajsPose[i][j], 
                    data.goalTrajsSupport[i][j]);
            } else if (type == OdometryRead) {
                odometry.update(
                    data.readTrajsPose[i][j], 
                    data.readTrajsSupport[i][j]);
            } else {
                throw std::logic_error("Invalid odometry type");
            }
            if (plot != nullptr) {
                odometryDebugRead->update(
                    data.readTrajsPose[i][j], 
                    data.readTrajsSupport[i][j]);
                odometryDebugGoal->update(
                    data.goalTrajsPose[i][j], 
                    data.goalTrajsSupport[i][j]);
                if (lastPhase > 0.8 && phase < 0.2) {
                    double enableGain = data.walkTrajsOrder[i][j](3);
                    odometryDebugOrder->updateFullStep(
                        2.0*data.walkTrajsOrder[i][j].segment(0, 3)
                        *enableGain);
                }
                plot->add(Leph::VectorLabel(
                    "odometry_x", odometry.state().x(),
                    "odometry_y", odometry.state().y(),
                    "identityRead_x", odometryDebugRead->state().x(),
                    "identityRead_y", odometryDebugRead->state().y(),
                    "identityGoal_x", odometryDebugGoal->state().x(),
                    "identityGoal_y", odometryDebugGoal->state().y(),
                    "identityOrder_x", odometryDebugOrder->state().x(),
                    "identityOrder_y", odometryDebugOrder->state().y()
                ));
            }
            lastPhase = phase;
        }
        if (plot != nullptr) {
            plot->add(Leph::VectorLabel(
                "target_x", data.targetDisplacements[i].x(),
                "target_y", data.targetDisplacements[i].y()
            ));
            std::cout 
                << "DistIdentityRead=" << 
                (data.targetDisplacements[i] - odometryDebugRead->state().segment(0, 2)).norm() 
                << " DistIdentityGoal=" << 
                (data.targetDisplacements[i] - odometryDebugGoal->state().segment(0, 2)).norm() 
                << " DistIdentityOrder=" << 
                (data.targetDisplacements[i] - odometryDebugOrder->state().segment(0, 2)).norm() 
                << " DistParameters=" << 
                (data.targetDisplacements[i] - odometry.state().segment(0, 2)).norm() 
                << std::endl;
            delete odometryDebugRead;
            delete odometryDebugOrder;
        }
        //Score resulting pose
        if (isLearningScore) {
            double tmpDist = 
                distanceFromArc(
                    odometry.state().x(),
                    odometry.state().y(),
                    data.targetDisplacements[i].x(),
                    data.targetDisplacements[i].y(),
                    AngularRangeFitness);
            if (tmpDist > 0.10) {
                error += pow(tmpDist, 2);
            }
        } else {
            error += 
                (data.targetDisplacements[i] - odometry.state().segment(0, 2))
                .squaredNorm();
        }
        count++;
    }
    if (count == 0) {
        return 0.0;
    } else {
        return sqrt(error/count);
    }
}

/**
 * Return default parameters for given model
 */
Eigen::VectorXd defaultParameters(
    Leph::OdometryModel::OdometryModelType model, 
    size_t indexStart, size_t indexEnd)
{
    Leph::OdometryModel odometry(model);
    return odometry.parameters();
}

/**
 * Optimize and return the best found odometry parameter
 * for given type and model using data sequnces between
 * given start and end indexes
 */
Eigen::VectorXd odometryModelOptimization(
    const OdometryData& data, OdometryType type, 
    Leph::OdometryModel::OdometryModelType model, 
    size_t indexStart, size_t indexEnd)
{
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&data, &type, &model, &indexStart, &indexEnd]
        (const Eigen::VectorXd& parameters) 
        {
            return odometryModelFitness(
                data, type, model, parameters, indexStart, indexEnd, true);
        };

    //Initial parameters
    Eigen::VectorXd initParams = defaultParameters(model, indexStart, indexEnd);
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(
        initParams, -1.0, CMAESLambda);
    cmaparams.set_quiet(CMAESQuiet);
    cmaparams.set_mt_feval(CMAESThreading);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(CMAESElitism);
    cmaparams.set_restarts(CMAESRestart);
    cmaparams.set_max_iter(CMAESMaxIteration);
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams);
    //Retrieve best parameters
    Eigen::VectorXd bestParameters = 
        cmasols.get_best_seen_candidate().get_x_dvec();

    return bestParameters;
}

int main(int argc, char** argv)
{
    //Parse arguments
    if (argc < 2) {
        std::cout << "./app OdometryLog1 [OdometryLog2] ..." << std::endl;
        return 1;
    }
    std::vector<std::string> filenames;
    for (size_t i=1;i<(size_t)argc;i++) {
        filenames.push_back(argv[i]);
    }

    //Initialize data structure
    OdometryData data;
    //Load data from logs
    for (size_t i=0;i<filenames.size();i++) {
        std::cout << "Loading data from " << filenames[i] << std::endl;
        loadDataFromFile(data, filenames[i]);
    }
    //Verbose
    std::cout << "Loaded " << data.readTrajsPose.size() << " sequences" << std::endl;
    for (size_t i=0;i<data.readTrajsPose.size();i++) {
        std::cout << "Seq " << i << " with " << data.readTrajsPose[i].size() << " points " 
            << "Displacement: " << data.targetDisplacements[i].transpose() << std::endl;
    }
    
    const OdometryType TypeOdometry = OdometryOrder;
    size_t sizeSequence = data.readTrajsPose.size();
    size_t maxSizeLearning = 3*sizeSequence/4;
    std::vector<std::pair<std::string,Leph::OdometryModel::OdometryModelType>> models = {
        {"ScalarX", Leph::OdometryModel::CorrectionScalarX},
        {"ScalarXY", Leph::OdometryModel::CorrectionScalarXY},
        {"ScalarXYZ", Leph::OdometryModel::CorrectionScalarXYZ},
        {"ProportionalXY", Leph::OdometryModel::CorrectionProportionalXY},
        {"ProportionalXYZ", Leph::OdometryModel::CorrectionProportionalXYZ},
        {"LinearSimpleXY", Leph::OdometryModel::CorrectionLinearSimpleXY},
        {"LinearSimpleXYZ", Leph::OdometryModel::CorrectionLinearSimpleXYZ},
        {"LinearFullXY", Leph::OdometryModel::CorrectionLinearFullXY},
        {"LinearFullXYZ", Leph::OdometryModel::CorrectionLinearFullXYZ},
    };

    for (const auto& it : models) {
        for (size_t i=1;i<=maxSizeLearning;i+=1) {
            for (size_t k=0;k<NumberTries;k++) {
                shuffleData(data);
                Eigen::VectorXd initParams = defaultParameters(it.second, 0, i);
                Eigen::VectorXd bestParams = odometryModelOptimization(
                    data, TypeOdometry, it.second, 0, i);
                double scoreInitialLearning = odometryModelFitness(
                    data, TypeOdometry, it.second, initParams, 0, i, true);
                double scoreLearning = odometryModelFitness(
                    data, TypeOdometry, it.second, bestParams, 0, i, true);
                double scoreInitialValidation = odometryModelFitness(
                    data, TypeOdometry, it.second, initParams, i+1, sizeSequence-1, false);
                double scoreValidation = odometryModelFitness(
                    data, TypeOdometry, it.second, bestParams, i+1, sizeSequence-1, false);
                if (IsPrintCSV) {
                    std::cout 
                        << it.first
                        << "," << (i+1)
                        << "," << (sizeSequence-1-i)
                        << "," << k
                        << "," << scoreInitialLearning
                        << "," << scoreLearning
                        << "," << scoreInitialValidation
                        << "," << scoreValidation
                        << std::endl;
                } else {
                    std::cout 
                        << it.first
                        << " nbLearn=" << (i+1)
                        << " nbTest=" << (sizeSequence-1-i)
                        << " try=" << k
                        << " initLearn=" << scoreInitialLearning
                        << " learn=" << scoreLearning
                        << " initTest=" << scoreInitialValidation
                        << " test=" << scoreValidation
                        << " dim=" << bestParams.size()
                        << " " << (scoreInitialValidation > scoreValidation ? "" : "!!!")
                        << " Params:" << bestParams.transpose()
                        << std::endl;
                }
                if (IsDebug) {
                    for (size_t j=maxSizeLearning+1;j<=sizeSequence-1;j++) {
                        displaySequence(data, j);
                        Leph::Plot plot;
                        odometryModelFitness(
                            data, TypeOdometry, it.second, bestParams, j, j, false, &plot);
                        plot
                            .plot("odometry_x", "odometry_y", Leph::Plot::LinesPoints, "index")
                            .plot("identityRead_x", "identityRead_y")
                            .plot("identityGoal_x", "identityGoal_y")
                            .plot("identityOrder_x", "identityOrder_y")
                            .plot("target_x", "target_y")
                            .render();
                    }
                }
            }
        }
    }

    return 0;
}

