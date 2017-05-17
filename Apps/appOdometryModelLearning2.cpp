#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "Model/HumanoidFixedModel.hpp"
#include "Odometry/Odometry.hpp"
#include "Calibration/LogLikelihoodMaximization.hpp"
#include "Utils/FileEigen.h"

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

// The whole configuration of the learner is included here
struct LearningConfig{
  OdometryType odometry_type;
  Leph::OdometryDisplacementModel::Type displacement_type;
  Leph::OdometryNoiseModel::Type noise_type;
  unsigned int sampling_number;
  unsigned int learning_set_size;
  int cmaes_elitism;
  int cmaes_max_iterations;
  int cmaes_restarts;
  int cmaes_lambda;
  int cmaes_sigma;
};

struct LearningConfig conf;

// Read next line as an OdometryType
OdometryType parseOdometryType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "OdometryOrder") return OdometryType::OdometryOrder;
  if (line == "OdometryGoal")  return OdometryType::OdometryGoal;
  if (line == "OdometryRead")  return OdometryType::OdometryRead;
  throw std::runtime_error("Unknown type of odometry: '" + line + "'");
}

// Should be in OdometryDisplacementModel
Leph::OdometryDisplacementModel::Type parseDisplacementType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "ProportionalXY" ) return Leph::OdometryDisplacementModel::Type::DisplacementProportionalXY;
  if (line == "ProportionalXYA") return Leph::OdometryDisplacementModel::Type::DisplacementProportionalXYA;
  if (line == "LinearSimpleXY" ) return Leph::OdometryDisplacementModel::Type::DisplacementLinearSimpleXY;
  if (line == "LinearSimpleXYA") return Leph::OdometryDisplacementModel::Type::DisplacementLinearSimpleXYA;
  if (line == "LinearFullXY"   ) return Leph::OdometryDisplacementModel::Type::DisplacementLinearFullXY;
  if (line == "LinearFullXYA"  ) return Leph::OdometryDisplacementModel::Type::DisplacementLinearFullXYA;
  throw std::runtime_error("Unknown type of displacement Model: '" + line + "'");
}

// Should be in OdometryNoiseModel
Leph::OdometryNoiseModel::Type parseNoiseType(std::istream & in)
{
  std::string line;
  std::getline(in,line);
  if (line == "Constant"    ) return Leph::OdometryNoiseModel::Type::NoiseConstant;
  if (line == "Proportional") return Leph::OdometryNoiseModel::Type::NoiseProportional;
  if (line == "LinearSimple") return Leph::OdometryNoiseModel::Type::NoiseLinearSimple;
  if (line == "LinearFull"  ) return Leph::OdometryNoiseModel::Type::NoiseLinearFull;
  throw std::runtime_error("Unknown type of noise Model: '" + line + "'");
}

/// Dirty code: read config
void readLearningConfig(const std::string & filename,
                        struct LearningConfig * conf)
{
  std::ifstream input(filename);
  conf->odometry_type = parseOdometryType(input);
  conf->displacement_type = parseDisplacementType(input);
  conf->noise_type = parseNoiseType(input);
  // TODO: produces silent error on invalid content, should be fixed
  input >> conf->sampling_number
        >> conf->learning_set_size
        >> conf->cmaes_elitism
        >> conf->cmaes_max_iterations
        >> conf->cmaes_restarts
        >> conf->cmaes_lambda
        >> conf->cmaes_sigma;
  input.close();
}

void writeLearningConfig(std::ostream & out, struct LearningConfig * conf)
{
  out << "OdometryType    : " << conf->odometry_type        << std::endl
      << "DisplacementType: " << conf->displacement_type    << std::endl
      << "NoiseType       : " << conf->noise_type           << std::endl
      << "SamplingNumber  : " << conf->sampling_number      << std::endl
      << "LearningSetSize : " << conf->learning_set_size    << std::endl
      << "Elitism         : " << conf->cmaes_elitism        << std::endl
      << "MaxIterations   : " << conf->cmaes_max_iterations << std::endl
      << "Restarts        : " << conf->cmaes_restarts       << std::endl
      << "Lambda          : " << conf->cmaes_lambda         << std::endl
      << "Sigma           : " << conf->cmaes_sigma          << std::endl;
}

/**
 * Odometry data for one 
 * recorded sequence
 */
struct OdometrySequence {
  //Model read and goal model cartesian 
  //pose in origin at each logged point
  //(X,Y,Theta).
  std::vector<Eigen::Vector3d> readTrajsPose;
  std::vector<Leph::HumanoidFixedModel::SupportFoot> readTrajsSupport;
  std::vector<Eigen::Vector3d> goalTrajsPose;
  std::vector<Leph::HumanoidFixedModel::SupportFoot> goalTrajsSupport;
  //Walk step, lateral, turn and phase 
  //walk order at each logged point 
  std::vector<Eigen::Vector4d> walkTrajsOrder;
  std::vector<double> walkTrajsPhase;
  //Target observed cartesian 
  //displacement  (X,Y,Theta)
  Eigen::Vector3d targetDisplacements;
};

/**
 * Append data loaded from fiven filename to
 * given OdometrySequence container
 */
static void loadOdometryDataFromFile(
  std::vector<OdometrySequence>& data, 
  const std::string& filename)
{
  //Open file
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error(
      "Unable to open log file: " + filename);
  }

  //Loop over file entries
  size_t lastSeq = -1;
  while (file.good() && file.peek() != EOF) {
    //Skip end of line
    while (file.peek() == ' ' || file.peek() == '\n') {
      file.ignore();
    }
    if (!file.good() || file.peek() == EOF) {
      break;
    }
    //Retrieve one file line
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
      //Start a new sequence
      data.push_back(OdometrySequence());
      data.back().targetDisplacements = 
        Eigen::Vector3d(targetX, targetY, targetA);
    }
    lastSeq = seq;
    data.back().readTrajsPose.push_back(
      Eigen::Vector3d(readPoseX, readPoseY, readPoseYaw));
    data.back().readTrajsSupport.push_back(
      (Leph::HumanoidFixedModel::SupportFoot)readSupportFoot);
    data.back().goalTrajsPose.push_back(
      Eigen::Vector3d(goalPoseX, goalPoseY, goalPoseYaw));
    data.back().goalTrajsSupport.push_back(
      (Leph::HumanoidFixedModel::SupportFoot)goalSupportFoot);
    data.back().walkTrajsOrder.push_back(
      Eigen::Vector4d(walkOrderX, walkOrderY, walkOrderTheta, walkOrderEnabled));
    data.back().walkTrajsPhase.push_back(
      walkPhase);
  }
  file.close();
}

/**
 * Build and return the initial 
 * parameters vector
 */
Eigen::VectorXd buildInitialParameters()
{
  Leph::Odometry odometry(conf.displacement_type, conf.noise_type);
  return odometry.getParameters();
}

/**
 * Build and return the 
 * normalization coefficients
 */
Eigen::VectorXd buildNormalizationCoef()
{
  Leph::Odometry odometry(conf.displacement_type, conf.noise_type);
  return odometry.getNormalization();
}

/**
 * Check and bound parameters to
 * acceptable range.
 * Return non zero cost if
 * parameters are unbounded.
 */
double boundParameters(
  const Eigen::VectorXd& params)
{
  Leph::Odometry odometry(
    conf.displacement_type, conf.noise_type);
  double cost = odometry.setParameters(params);
  if (cost > 0.0) {
    return 1000.0 + 1000.0*cost;
  } else {
    return 0.0;
  }
}

/**
 * Model initialization function
 */
Leph::Odometry initModel(const Eigen::VectorXd& params)
{
  Leph::Odometry odometry(conf.displacement_type, conf.noise_type);
  double cost = odometry.setParameters(params);
  if (cost > 0.0) {
    std::cout << "Parameters: " 
              << params.transpose() << std::endl;
    throw std::logic_error("Parameters out of bounds");
  }

  return odometry;
}

/**
 * Compute and return the estimation
 * (try to predict observation) from
 * given data and using given parameters
 */
Eigen::VectorXd evaluateParameters(
  const Eigen::VectorXd& params,
  const OdometrySequence& data,
  bool noRandom,
  Leph::Odometry& odometry,
  std::default_random_engine& engine)
{
  (void)params;

  odometry.reset();

  std::default_random_engine* usedEngine = &engine;
  if (noRandom) {
    //Disable the random engien to disable
    //the noise model in odometry
    usedEngine = nullptr;
  }

  //Iterate over recorded point inside the sequence
  double lastPhase = data.walkTrajsPhase.front();
  for (size_t i=0;i<data.readTrajsPose.size();i++) {
    double phase = data.walkTrajsPhase[i];
    //Use given odometry type
    if (conf.odometry_type == OdometryOrder) {
      if (lastPhase > 0.8 && phase < 0.2) {
        double enableGain = data.walkTrajsOrder[i](3);
        odometry.updateFullStep(
          2.0*data.walkTrajsOrder[i].segment(0, 3)
          *enableGain,
          usedEngine);
      }
    } else if (conf.odometry_type == OdometryGoal) {
      odometry.update(
        data.goalTrajsPose[i], 
        data.goalTrajsSupport[i],
        usedEngine);
    } else if (conf.odometry_type == OdometryRead) {
      odometry.update(
        data.readTrajsPose[i], 
        data.readTrajsSupport[i],
        usedEngine);
    } else {
      throw std::logic_error("Invalid odometry type");
    }
    lastPhase = phase;
  }

  //Final integrated state
  Eigen::VectorXd estimate(3);
  estimate << 
    odometry.state().x(),
    odometry.state().y(),
    odometry.state().z();
  return estimate;
}

/**
 * Calibrate the Odometry correction and noise 
 * parameters using log likelihood maximisation 
 * through CMA-ES optimization.
 */
int main(int argc, char** argv)
{
  if (argc != 4) {
    std::cout << "Usage: ./app "
              << "odometry.data learningConfig.conf outputPrefix" 
              << std::endl;
    return 1;
  }
  std::string logPath = argv[1];
  std::string confPath = argv[2];
  std::string outPath = argv[3];

  //Loading config
  readLearningConfig(confPath, &conf);
  writeLearningConfig(std::cout, &conf);
    
  //Loading log sequences
  std::vector<OdometrySequence> logs;
  loadOdometryDataFromFile(logs, logPath);
  std::cout << "Loading odometry data from: " 
            << logPath << " with " 
            << logs.size()
            << " sequences" << std::endl;

  //Parameters initialization
  Eigen::VectorXd initParams = buildInitialParameters();
    
  //Initialize the logLikelihood 
  //maximisation process
  Leph::LogLikelihoodMaximization
    <OdometrySequence, Leph::Odometry> calibration;
  calibration.setInitialParameters(
    initParams, buildNormalizationCoef());
  calibration.setUserFunctions(evaluateParameters, boundParameters, initModel);
  //Add observations data
  for (size_t i=0;i<logs.size();i++) {
    Eigen::VectorXd obs(3);
    obs << 
      logs[i].targetDisplacements.x(), 
      logs[i].targetDisplacements.y(), 
      Leph::AngleBound(
        -logs[i].targetDisplacements.z()*2.0*M_PI/12.0);
    calibration.addObservation(obs, logs[i]);
  }
    
  //Start the CMA-ES optimization
  calibration.runOptimization(
    conf.sampling_number, 
    conf.learning_set_size,
    conf.cmaes_max_iterations, 
    conf.cmaes_restarts, 
    conf.cmaes_lambda, 
    conf.cmaes_sigma, 
    conf.cmaes_elitism,
    nullptr);
    
  //Display the best found parameters
  Eigen::VectorXd bestParams = calibration.getParameters();
  Leph::Odometry tmpOdometry = initModel(bestParams);
  tmpOdometry.printParameters();
  //Export the optimized model parameters
  //Write to file
  std::string odoParamsPath = outPath + "odometryModel.params";
  std::cout << "Writing odometry parameters to: " 
            << odoParamsPath << std::endl;
  std::ofstream file(odoParamsPath);
  if (!file.is_open()) {
    throw std::runtime_error(
      "Unable to open file: " + odoParamsPath);
  }
  file 
    << (int)tmpOdometry.getDisplacementType() << " " 
    << (int)tmpOdometry.getNoiseType() << std::endl;
  Leph::WriteEigenVectorToStream(file, bestParams);
  file.close();

  return 0;
}

