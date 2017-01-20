#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include "TrajectoryGeneration/TrajectoryParameters.hpp"
#include "TrajectoryGeneration/TrajectoryGeneration.hpp"
#include "TrajectoryDefinition/CommonTrajs.h"
#include "Utils/FileVector.h"
#include "TrajectoryDefinition/TrajKickSingle.hpp"
#include "TrajectoryDefinition/TrajKickDouble.hpp"
#include "TrajectoryDefinition/TrajLegLift.hpp"
#include "TrajectoryDefinition/TrajWalk.hpp"

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
        std::cout << "-- kicksingle" << std::endl;
        std::cout << "-- kickdouble" << std::endl;
        std::cout << "-- leglift" << std::endl;
        std::cout << "-- walk" << std::endl;
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
    //Load trajectory template
    if (trajName == "kicksingle") {
        Leph::TrajKickSingle::initializeParameters(trajParams);
        generator.setTrajectoryGenerationFunc(Leph::TrajKickSingle::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajKickSingle::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajKickSingle::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajKickSingle::funcCheckDOF(trajParams));
        generator.setScoreFunc(Leph::TrajKickSingle::funcScore(trajParams));
        generator.setEndScoreFunc(Leph::TrajKickSingle::funcEndScore(trajParams));
    } else if (trajName == "kickdouble") {
        Leph::TrajKickDouble::initializeParameters(trajParams);
        generator.setTrajectoryGenerationFunc(Leph::TrajKickDouble::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajKickDouble::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajKickDouble::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajKickDouble::funcCheckDOF(trajParams));
        generator.setScoreFunc(Leph::TrajKickDouble::funcScore(trajParams));
        generator.setEndScoreFunc(Leph::TrajKickDouble::funcEndScore(trajParams));
    } else if (trajName == "leglift") {
        Leph::TrajLegLift::initializeParameters(trajParams);
        generator.setTrajectoryGenerationFunc(Leph::TrajLegLift::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajLegLift::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajLegLift::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajLegLift::funcCheckDOF(trajParams));
        generator.setScoreFunc(Leph::TrajLegLift::funcScore(trajParams));
        generator.setEndScoreFunc(Leph::TrajLegLift::funcEndScore(trajParams));
    } else if (trajName == "walk") {
        Leph::TrajWalk::initializeParameters(trajParams);
        generator.setTrajectoryGenerationFunc(Leph::TrajWalk::funcGeneration(trajParams));
        generator.setCheckParametersFunc(Leph::TrajWalk::funcCheckParams(trajParams));
        generator.setCheckStateFunc(Leph::TrajWalk::funcCheckState(trajParams));
        generator.setCheckDOFFunc(Leph::TrajWalk::funcCheckDOF(trajParams));
        generator.setScoreFunc(Leph::TrajWalk::funcScore(trajParams));
        generator.setEndScoreFunc(Leph::TrajWalk::funcEndScore(trajParams));
    } else {
        std::cout << "Invalid trajectory name: " << trajName << std::endl;
        return 1;
    }

    //Build initial parameters
    Eigen::VectorXd initParams = trajParams.buildVector();
    //Build normalization coefficents
    Eigen::VectorXd normCoefs = trajParams.buildNormalizationCoefs();
    
    //Insert inputs parameters to trajectory parameter
    std::string paramsStr = "_";
    for (const auto& it : inputParameters) {
        std::cout << "Custom Parameter: " << it.first << "=" << it.second << std::endl;
        trajParams.set(it.first) = it.second;
        //Skip CMAES parameters
        if (it.first.find("cmaes") == std::string::npos) {
            paramsStr += it.first + std::string("_") 
                + std::to_string(it.second) + std::string("_");
        }
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
    //Set normalization coefficients
    generator.setNormalizationCoefs(normCoefs);
    
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
        trajParams.get("cmaes_sigma"),
        (unsigned int)trajParams.get("cmaes_elitism"));

#ifdef LEPH_VIEWER_ENABLED
    //Display found trajectory
    TrajectoriesDisplay(generator.bestTrajectories());
#endif

    return 0;
}

