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
 * Global configuration
 */
static const unsigned int CMAESMaxIteration = 5000;
static const unsigned int CMAESLambda = 10;
static const unsigned int CMAESRestart = 3;
static const bool CMAESQuiet = true;
static const bool CMAESElitism = true;
static const bool CMAESThreading = true;
static const unsigned int NumberTries = 100;
static const double AngularRangeFitness = 5.0*M_PI/180.0;
static const bool IsPrintCSV = true;
static const bool IsDebug = false;
static const double AngularErrorCoef = 0.57*0.2;
static const OdometryType TypeOdometry = OdometryOrder;
static const size_t StartNumberLearnSeqs = 4;
static const size_t IncrNumberLearnSeqs = 5;
static const size_t NumberValidationSeqs = 5;

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
    std::vector<Eigen::Vector3d> targetDisplacements;
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
        double targetA;
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
        file >> targetA;
        if (lastSeq != seq) {
            data.readTrajsPose.push_back(std::vector<Eigen::Vector3d>());
            data.readTrajsSupport.push_back(std::vector<Leph::HumanoidFixedModel::SupportFoot>());
            data.goalTrajsPose.push_back(std::vector<Eigen::Vector3d>());
            data.goalTrajsSupport.push_back(std::vector<Leph::HumanoidFixedModel::SupportFoot>());
            data.walkTrajsOrder.push_back(std::vector<Eigen::Vector4d>());
            data.walkTrajsPhase.push_back(std::vector<double>());
            data.targetDisplacements.push_back(Eigen::Vector3d(targetX, targetY, targetA));
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
    if (
        dist1 <= dist2 && 
        dist1 <= dist3 && 
        fabs(Leph::AngleDistance(angle, targetAngle)) <= angularRange
    ) {
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
                (data.targetDisplacements[i].segment(0, 2) 
                    - odometryDebugRead->state().segment(0, 2)).norm() 
                << " DistIdentityGoal=" << 
                (data.targetDisplacements[i].segment(0, 2)
                    - odometryDebugGoal->state().segment(0, 2)).norm() 
                << " DistIdentityOrder=" << 
                (data.targetDisplacements[i].segment(0, 2) 
                    - odometryDebugOrder->state().segment(0, 2)).norm() 
                << " DistParameters=" << 
                (data.targetDisplacements[i].segment(0, 2) 
                    - odometry.state().segment(0, 2)).norm() 
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
            double finalAngle = odometry.state().z();
            double targetAngle = -data.targetDisplacements[i].z()*2.0*M_PI/12.0;
            double errorAngle = fabs(Leph::AngleDistance(targetAngle, finalAngle));
            error += pow(tmpDist, 2);
            if (
                data.targetDisplacements[i].z() >= 0.0 && 
                errorAngle > 2.0*M_PI/12.0
            ) {
                error += pow(AngularErrorCoef*errorAngle, 2);
            }
        } else {
            error += 
                (data.targetDisplacements[i].segment(0, 2) 
                    - odometry.state().segment(0, 2))
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
    Leph::OdometryModel::OdometryModelType model)
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
    Eigen::VectorXd initParams = defaultParameters(model);
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
        std::cout << "./app OdometryLearnLog1 ... NEXT OdometryTestLogX ..." << std::endl;
        return 1;
    }
    bool isMultipleLogSet = false;
    std::vector<std::string> filenames1;
    std::vector<std::string> filenames2;
    size_t indexFile = 1;
    while (indexFile < (size_t)argc) {
        if (std::string(argv[indexFile]) == "NEXT") {
            indexFile++;
            break;
        } else {
            filenames1.push_back(argv[indexFile]);
        }
        indexFile++;
    }
    for (size_t i=indexFile;i<(size_t)argc;i++) {
        isMultipleLogSet = true;
        filenames2.push_back(argv[i]);
    }

    //Initialize data structure
    OdometryData data1;
    OdometryData data2;
    //Load data from logs
    for (size_t i=0;i<filenames1.size();i++) {
        if (isMultipleLogSet) {
            std::cout << "Loading learn data from " << filenames1[i] << std::endl;
        } else {
            std::cout << "Loading data from " << filenames1[i] << std::endl;
        }
        loadDataFromFile(data1, filenames1[i]);
    }
    for (size_t i=0;i<filenames2.size();i++) {
        std::cout << "Loading test  data from " << filenames2[i] << std::endl;
        loadDataFromFile(data2, filenames2[i]);
    }
    //Verbose loading
    if (isMultipleLogSet) {
        std::cout << "Loaded learn " << data1.readTrajsPose.size() << " sequences" << std::endl;
        for (size_t i=0;i<data1.readTrajsPose.size();i++) {
            std::cout << "Learn Seq " << i << " with " << data1.readTrajsPose[i].size() << " points " 
                << "Displacement: " << data1.targetDisplacements[i].transpose() << std::endl;
        }
        std::cout << "Loaded test " << data2.readTrajsPose.size() << " sequences" << std::endl;
        for (size_t i=0;i<data2.readTrajsPose.size();i++) {
            std::cout << "Test Seq " << i << " with " << data2.readTrajsPose[i].size() << " points " 
                << "Displacement: " << data2.targetDisplacements[i].transpose() << std::endl;
        }
    } else {
        std::cout << "Loaded " << data1.readTrajsPose.size() << " sequences" << std::endl;
        for (size_t i=0;i<data1.readTrajsPose.size();i++) {
            std::cout << "Seq " << i << " with " << data1.readTrajsPose[i].size() << " points " 
                << "Displacement: " << data1.targetDisplacements[i].transpose() << std::endl;
        }
    }
    
    //Experimented Odometry models
    std::vector<std::pair<std::string,Leph::OdometryModel::OdometryModelType>> models = {
        //{"ScalarX", Leph::OdometryModel::CorrectionScalarX},
        //{"ScalarXY", Leph::OdometryModel::CorrectionScalarXY},
        //{"ScalarXYA", Leph::OdometryModel::CorrectionScalarXYA},
        //{"ProportionalXY", Leph::OdometryModel::CorrectionProportionalXY},
        //{"ProportionalXYA", Leph::OdometryModel::CorrectionProportionalXYA},
        //{"LinearSimpleXY", Leph::OdometryModel::CorrectionLinearSimpleXY},
        {"LinearSimpleXYA", Leph::OdometryModel::CorrectionLinearSimpleXYA},
        //{"LinearFullXY", Leph::OdometryModel::CorrectionLinearFullXY},
        {"LinearFullXYA", Leph::OdometryModel::CorrectionLinearFullXYA},
        //{"ProportionalHistoryXY", Leph::OdometryModel::CorrectionProportionalHistoryXY},
        //{"ProportionalHistoryXYA", Leph::OdometryModel::CorrectionProportionalHistoryXYA},
        //{"LinearSimpleHistoryXY", Leph::OdometryModel::CorrectionLinearSimpleHistoryXY},
        {"LinearSimpleHistoryXYA", Leph::OdometryModel::CorrectionLinearSimpleHistoryXYA},
        //{"LinearFullHistoryXY", Leph::OdometryModel::CorrectionLinearFullHistoryXY},
        {"LinearFullHistoryXYA", Leph::OdometryModel::CorrectionLinearFullHistoryXYA},
    };

    //Define first and last learning 
    //and validation index
    size_t learnStartIndex;
    size_t learnEndIndex;
    size_t testStartIndex;
    size_t testEndIndex;
    if (isMultipleLogSet) {
        learnStartIndex = 0;
        learnEndIndex = data1.readTrajsPose.size() - 1;
        testStartIndex = 0;
        testEndIndex = data2.readTrajsPose.size() - 1;
    } else {
        size_t tmpSize = data1.readTrajsPose.size();
        learnStartIndex = 0;
        learnEndIndex = tmpSize - NumberValidationSeqs - 1;
        testStartIndex = tmpSize - NumberValidationSeqs;
        testEndIndex = tmpSize - 1;
    }
    //Print CSV headers
    if (IsPrintCSV) {
        std::cout << 
            "OdometryModelName,CountLearn,CountTest,Try,RESULTS,InitLearn,Learn,InitTest,Test" 
            << std::endl;
        std::cout << 
            "OdometryModelName,CountLearn,CountTest,Try,TEST,Index,InitTest,Test" 
            << std::endl;
        std::cout << 
            "OdometryModelName,CountLearn,CountTest,Try,PARAMS,SizeParams,..." 
            << std::endl;
    }
    //Loop over odometry models
    for (const auto& it : models) {
        //Loop over the number of sequences used to learn
        for (size_t i=StartNumberLearnSeqs;i<=learnEndIndex;i+=IncrNumberLearnSeqs) {
            //Loop over many tries
            for (size_t k=0;k<NumberTries;k++) {
                //Randomize the data
                shuffleData(data1);
                shuffleData(data2);
                OdometryData* dataLearn = nullptr;
                OdometryData* dataTest = nullptr;
                if (isMultipleLogSet) {
                    dataLearn = &data1;
                    dataTest = &data2;
                } else {
                    dataLearn = &data1;
                    dataTest = &data1;
                }
                //Retrieve identity initial parameters
                Eigen::VectorXd initParams = defaultParameters(it.second);
                //Optimize model parameters on data learning set
                Eigen::VectorXd bestParams = odometryModelOptimization(
                    *dataLearn, TypeOdometry, it.second, learnStartIndex, i);
                //Compute RMSE score on learning and validation set
                double scoreInitialLearning = odometryModelFitness(
                    *dataLearn, TypeOdometry, it.second, initParams, learnStartIndex, i, true);
                double scoreLearning = odometryModelFitness(
                    *dataLearn, TypeOdometry, it.second, bestParams, learnStartIndex, i, true);
                double scoreInitialValidation = odometryModelFitness(
                    *dataTest, TypeOdometry, it.second, initParams, testStartIndex, testEndIndex, false);
                double scoreValidation = odometryModelFitness(
                    *dataTest, TypeOdometry, it.second, bestParams, testStartIndex, testEndIndex, false);
                //Print CSV
                if (IsPrintCSV) {
                    //Print summary
                    std::cout 
                        << it.first
                        << "," << i-learnStartIndex+1
                        << "," << testEndIndex-testStartIndex+1
                        << "," << k
                        << ",RESULTS"
                        << "," << scoreInitialLearning
                        << "," << scoreLearning
                        << "," << scoreInitialValidation
                        << "," << scoreValidation
                        << std::endl;
                    //Evaluate each validation sequence independendly
                    //without computing the RMSE
                    for (size_t j=testStartIndex;j<=testEndIndex;j++) {
                        double scoreInitialValidationSingle = odometryModelFitness(
                            *dataTest, TypeOdometry, it.second, initParams, j, j, false);
                        double scoreValidationSingle = odometryModelFitness(
                            *dataTest, TypeOdometry, it.second, bestParams, j, j, false);
                        std::cout
                            << it.first
                            << "," << i-learnStartIndex+1
                            << "," << testEndIndex-testStartIndex+1
                            << "," << k
                            << ",TEST"
                            << "," << j-testStartIndex+1
                            << "," << scoreInitialValidationSingle 
                            << "," << scoreValidationSingle 
                            << std::endl;
                    }
                    //Print best found parameters
                    std::cout 
                        << it.first
                        << "," << i-learnStartIndex+1
                        << "," << testEndIndex-testStartIndex+1
                        << "," << k
                        << ",PARAMS"
                        << "," << bestParams.size();
                    for (size_t j=0;j<(size_t)bestParams.size();j++) {
                        std::cout << ",";
                        std::cout << bestParams(j);
                    }
                    std::cout << std::endl;
                } else {
                    //Print textual summary
                    std::cout 
                        << it.first
                        << " nbLearn=" << i-learnStartIndex+1
                        << " nbTest=" << testEndIndex-testStartIndex+1
                        << " try=" << k
                        << " initLearn=" << scoreInitialLearning
                        << " learn=" << scoreLearning
                        << " initTest=" << scoreInitialValidation
                        << " test=" << scoreValidation
                        << " dim=" << bestParams.size()
                        << " " << (scoreInitialValidation > scoreValidation ? "" : "!!!")
                        << " Params:" << std::endl << bestParams
                        << std::endl;
                }
                //Show trajectory of validation set
                if (IsDebug) {
                    for (size_t j=testStartIndex;j<=testEndIndex;j++) {
                        displaySequence(*dataTest, j);
                        Leph::Plot plot;
                        odometryModelFitness(
                            *dataTest, TypeOdometry, it.second, bestParams, j, j, false, &plot);
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

