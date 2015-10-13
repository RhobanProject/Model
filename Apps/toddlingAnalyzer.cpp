#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/PressureModel.hpp"
#include "Utils/Scheduling.hpp"

#include "Model/ModelBuilder.hpp"

using namespace Leph;

/**
 * New version of the toddling analyzer, multiple output files, prefixed by analyzed_,
 * Each line contains only the information for a given time
 */

int main(int argc, char** argv)
{
    //Command line arguments
    if (argc < 2) {
        std::cerr << "Usage: ./app <logFile1> <logFile2> ..." << std::endl;
        return -1;
    }

    // The basis for which data should be plotted
    std::vector<std::string> requiredBasis = {"origin",
                                              "COM",
                                              "left_arch_center",
                                              "right_arch_center"};
    std::vector<std::string> targets = {"COM","COP",
                                        "right_arch_center",
                                        "left_arch_center"};


    for (int argNo = 1; argNo < argc; argNo++) {
      std::string logsFile = argv[argNo];
      std::cout << "Loading " << logsFile << std::endl;

      //Initialize model instances (restart between two logs)
      Leph::PressureModel model(generateGrobanWithToe(true));

      //Loading data
      Leph::MatrixLabel logs;
      logs.load(logsFile);

      MatrixLabel outputData;

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
      std::map<std::string, std::map<std::string, Eigen::Vector3d>> positions;

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
          for (const std::string& target : targets) {
            if (target == basis) continue;
            if (target == "COM") {
              positions[basis]["COM"] = model.centerOfMass(basis);
            }
            else if (target == "COP") {
              if (basis == "COM") {
                Eigen::Vector3d copInOrigin = model.getCOP("origin");
                positions[basis]["COP"] = model.getPosInCOMBasis("origin", copInOrigin);
              }
              else {
                positions[basis]["COP"] = model.getCOP(basis);
              }
            }
            else {
              if (basis == "COM") {
                positions[basis][target] = model.getPosInCOMBasis(target);
              }
              else {
                positions[basis][target] = model.position(target, basis);
              }
            }
          }
        }

        VectorLabel data;
        data.append("phase", logs[indexLog]("phase"));
        data.append("targetX", logs[indexLog]("targetComX"));
        data.append("targetY", logs[indexLog]("targetComY"));
        for (const std::string& basis : requiredBasis) {
          for (const std::string& target : targets) {
            if (target == basis) continue;
            std::string colPrefix = basis + ":" + target;
            Eigen::Vector3d pos = positions[basis][target];
            data.append(colPrefix + ":x", pos.x());
            data.append(colPrefix + ":y", pos.y());
            data.append(colPrefix + ":z", pos.z());
          }
        }
        outputData.append(data);
      }

      // Computing dst name
      size_t index = logsFile.find_last_of("/\\");
      std::string path, file;
      if (index == std::string::npos) {
        path = "";
        file = logsFile;
      }
      else {
        path = logsFile.substr(0, index + 1);
        file = logsFile.substr(index + 1);
      }
      std::string outputFile = path + "analyzed_" + file;
      outputData.save(outputFile);

      std::cout << "Analyze data written in: '" << outputFile << "'" << std::endl;
    }

    return 0;
}

