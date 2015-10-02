#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/PressureModel.hpp"
#include "Utils/Scheduling.hpp"

#include "Model/ModelBuilder.hpp"

using namespace Leph;

int main(int argc, char** argv)
{
    //Command line arguments
    if (argc < 3) {
        std::cerr << "Usage: ./app <dstFile> <logFile1> <logFile2> ..." << std::endl;
        return -1;
    }

    // The basis for which data should be plotted
    std::vector<std::string> requiredBasis = {"origin",
                                              "left_arch_center",
                                              "right_arch_center"};
    std::vector<std::string> targets = {"COM","COP","right_arch_center",};


    MatrixLabel outputData;
    std::string logsFile = argv[1];

    //Initialize model instances
    Leph::PressureModel model(generateGrobanWithToe(true));

    for (int argNo = 2; argNo < argc; argNo++) {
      std::string logsFile = argv[argNo];
      std::cout << "Loading " << logsFile << std::endl;

      //Loading data
      Leph::MatrixLabel logs;
      logs.load(logsFile);

      //Print data informations
      std::cout << "Loaded " 
                << logs.size() << " points with " 
                << logs.dimension() << " entries" << std::endl;
      if (logs.size() == 0) {
        std::cout << "\tSkipping empty log" << std::endl;
        continue;
      }

      //Initialize DOF vector
      Leph::VectorLabel motorsDOF  = logs[0].extract("pos").rename("pos", "");


      // Store positions: by basis and then by target
      std::map<std::string, std::map<std::string, Eigen::Vector3d>> oldPositions;
      std::map<std::string, std::map<std::string, Eigen::Vector3d>> newPositions;

      //Analyzing Log
      for (size_t indexLog = 0; indexLog < logs.size(); indexLog++) {

        //Assign DOF
        motorsDOF.assignOp(logs[indexLog], "pos", "");
        model.setDOF(motorsDOF, true);
        // Assign pressure
        Leph::VectorLabel pressures = logs[indexLog].extract("pressure").rename("pressure", "");
        pressures = pressures.renameLabels(":val:","_gauge_");

        model.updatePressure(pressures);

        // TODO updateBase might need some improvement
        model.updateBase();


        for (const std::string& basis : requiredBasis) {
          newPositions[basis]["COM"] = model.centerOfMass(basis);
          newPositions[basis]["COP"] = model.getCOP(basis);
        }

        if (indexLog > 1) {
          VectorLabel data;
          data.append("srcPhase", logs[indexLog-1]("phase"));
          data.append("nextPhase", logs[indexLog]("phase"));
          data.append("targetY", logs[indexLog-1]("targetComY"));
          for (const std::string& basis : requiredBasis) {
            for (const std::string& target : targets) {
              std::string colSuffix = ":" + basis + ":" + target;
              Eigen::Vector3d newPos = newPositions[basis][target];
              Eigen::Vector3d oldPos = oldPositions[basis][target];
              data.append("src"  + colSuffix + ":x", newPos.x());
              data.append("src"  + colSuffix + ":y", newPos.y());
              data.append("src"  + colSuffix + ":z", newPos.z());
              data.append("next" + colSuffix + ":x", oldPos.x());
              data.append("next" + colSuffix + ":y", oldPos.y());
              data.append("next" + colSuffix + ":z", oldPos.z());
            }
          }
          outputData.append(data);
        }
        for (const std::string& basis : requiredBasis) {
          for (const std::string& target : targets) {
              oldPositions[basis][target] = newPositions[basis][target];
          }
        }
      }
    }

    std::string dstFile = argv[1];
    outputData.save(dstFile);

    return 0;
}

