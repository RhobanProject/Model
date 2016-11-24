#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include "TrajectoryGeneration/TrajectoryParameters.hpp"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryDefinition/DefaultTrajParameters.h"
#include "Utils/FileVector.h"
#include "TrajectoryDefinition/TrajKick.hpp"
#include "TrajectoryDefinition/TrajLegLift.hpp"

#ifdef LEPH_VIEWER_ENABLED
#include "TrajectoryGeneration/TrajectoryDisplay.h"
#endif

int main(int argc, char** argv)
{
    //Parse argument
    if (argc < 4) {
        std::cout << "Usage: ./app RUN trajectoryName outputPrefix [paramName=value] ..." << std::endl;
        std::cout << "Usage: ./app SEED trajectoryName outputPrefix restartParameters [paramName=value] ..." << std::endl;
        std::cout << "Available trajectories:" << std::endl;
        std::cout << "-- kick" << std::endl;
        std::cout << "-- leglift" << std::endl;
        return 1;
    }
    std::string mode = argv[1];
    if (mode != "RUN" && mode != "SEED") {
        std::cout << "Invalid mode: " << mode << std::endl;
        return 1;
    }
    std::string trajName = argv[2];
    std::string outputPrefix = argv[3];
    std::string seedParametersFile = "";
    size_t startInputIndex = 4;
    if (mode == "SEED") {
        if (argc < 5) {
            std::cout << "SEED mode but no parameters file" << std::endl;
            return 1;
        }
        startInputIndex = 5;
        seedParametersFile = argv[4];
    }
    //Parse parameters
    std::vector<std::pair<std::string, double>> inputParameters;
    for (size_t i=startInputIndex;i<(size_t)argc;i++) {
        std::string part = argv[i];
        size_t pos = part.find("=");
        if (pos == std::string::npos) {
            std::cout << "Error format: " << part << std::endl;
            return 1;
        }
        std::string partName = part.substr(0, pos);
        std::string partVal = part.substr(pos+1);
        if (partName.size() == 0 || partVal.size() == 0) {
            std::cout << "Error format: " << part << std::endl;
            return 1;
        }
        inputParameters.push_back({partName, std::stod(partVal)});
    }

    //Initialize trajectory parameters
    Leph::TrajectoryParameters trajParams = Leph::DefaultTrajParameters();

    //Initialize the generator
    Leph::TrajectoryGeneration generator(Leph::SigmabanModel);
    Eigen::VectorXd initParams;
    //Load trajectory template
    if (trajName == "kick") {
        initParams = Leph::TrajKick::initialParameters(trajParams);
        generator.setTrajectoryGenerationFunc(Leph::TrajKick::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajKick::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajKick::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajKick::funcCheckDOF(trajParams));
        generator.setScoreFunc(Leph::TrajKick::funcScore(trajParams));
        generator.setEndScoreFunc(Leph::TrajKick::funcEndScore(trajParams));
    } else if (trajName == "leglift") {
        initParams = Leph::TrajLegLift::initialParameters(trajParams);
        generator.setTrajectoryGenerationFunc(Leph::TrajLegLift::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajLegLift::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajLegLift::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajLegLift::funcCheckDOF(trajParams));
        generator.setScoreFunc(Leph::TrajLegLift::funcScore(trajParams));
        generator.setEndScoreFunc(Leph::TrajLegLift::funcEndScore(trajParams));
    } else {
        std::cout << "Invalid trajectory name: " << trajName << std::endl;
        return 1;
    }
    
    //Insert inputs parameters to trajectory parameter
    std::string paramsStr = "_";
    for (const auto& it : inputParameters) {
        std::cout << "Custom Parameter: " << it.first << "=" << it.second << std::endl;
        trajParams.set(it.first) = it.second;
        paramsStr += it.first + std::string("_") 
            + std::to_string(it.second) + std::string("_");
    }

    //Verbose
    char hostnameStr[100];
    gethostname(hostnameStr, 100);
    std::string hostname = hostnameStr;
    std::string filename = 
        outputPrefix + trajName + paramsStr +
        hostname + "_" + Leph::currentDate();
    std::cout << "Hostname: " << hostname << std::endl;
    std::cout << "Output path: " << filename << std::endl;
    std::cout << "Starting with mode=" << mode << " and dimension=" << initParams.size() << std::endl;
    std::cout << "CMA-ES"
        << " max_iterations=" << (unsigned int)trajParams.get("cmaes_max_iterations")
        << " restarts=" << (unsigned int)trajParams.get("cmaes_restarts")
        << " lambda=" << (unsigned int)trajParams.get("cmaes_lambda")
        << " sigma=" << trajParams.get("cmaes_sigma")
        << std::endl;

    //Load initial parameters if SEED mode
    if (mode == "SEED") {
        Eigen::VectorXd tmpVect = Leph::ReadVector(seedParametersFile);
        if (tmpVect.size() != initParams.size()) {
            std::cout << "Invalid seed parameters size: " 
                << std::to_string(tmpVect.size()) << std::endl;
            return 1;
        } else {
            std::cout << "Seed parameters loaded from: " << seedParametersFile << std::endl;
            initParams = tmpVect;
        }
    } 

    //Set initial parameters
    generator.setInitialParameters(initParams);
    
#ifdef LEPH_VIEWER_ENABLED
    //Display initial trajectory
    TrajectoriesDisplay(generator.generateTrajectory(generator.initialParameters()));
#endif
    
    //Run the CMA-ES optimization
    generator.runOptimization(
        (unsigned int)trajParams.get("cmaes_max_iterations"),
        (unsigned int)trajParams.get("cmaes_restarts"),
        filename, 
        (unsigned int)trajParams.get("cmaes_lambda"), 
        trajParams.get("cmaes_sigma"));

#ifdef LEPH_VIEWER_ENABLED
    //Display found trajectory
    TrajectoriesDisplay(generator.bestTrajectories());
#endif

    return 0;
}

