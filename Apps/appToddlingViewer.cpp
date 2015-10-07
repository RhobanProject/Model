#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/PressureModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"

#include "Model/ModelBuilder.hpp"
#include "Utils/Chrono.hpp"
#include "Utils/STLibrary.hpp"

using namespace Leph;

class ToddlingViewer : public ModelViewer {
public:

  ToddlingViewer() : ModelViewer(1200,900) {}

  size_t indexLog = 0;
  size_t sizeLog = 0;

  bool update() override
    {
      //Handle keyboard without spam
      sf::Event event;
      while (_window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          return false;
        }
        if (event.type == sf::Event::KeyPressed) {
          switch (event.key.code) {
          case sf::Keyboard::Escape:
            return false;
          case sf::Keyboard::I:
            std::cout << "IndexLog: " << (indexLog + 1) << "/" << sizeLog << std::endl;;
            break;
          case sf::Keyboard::N:
            indexLog++;
            break;
          case sf::Keyboard::P:
            indexLog--;
            break;
          case sf::Keyboard::PageUp:
            indexLog += 50;
            break;
          case sf::Keyboard::PageDown:
            indexLog -= 50;
            break;
          default:
            break;
          }
          indexLog = std::min(sizeLog,std::max((size_t)0, indexLog));
        }
      }
      return ModelViewer::update();
    }


};

int main(int argc, char** argv)
{
  //Command line arguments
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: ./app <logsFile> <opt:startIndex>" << std::endl;
    return -1;
  }

  std::string logsFile = argv[1];
  std::cout << "Loading " << logsFile << std::endl;


  size_t startIndex = 0;

  if (argc > 2) {
    startIndex = std::stoi(argv[2]);
  }

  //Loading data
  Leph::MatrixLabel logs;
  logs.load(logsFile);

  //Print data informations
  std::cout << "Loaded " 
            << logs.size() << " points with " 
            << logs.dimension() << " entries" << std::endl;
  if (logs.size() == 0) {
    return 0;
  }
    

  //Initialize model instances
  Leph::PressureModel model(generateGrobanWithToe(true));


  //Initialize DOF vector
  Leph::VectorLabel motorsDOF  = logs[0].extract("pos").rename("pos", "");

  //Main loop
  size_t indexLog = 0;
  ToddlingViewer viewer;
  viewer.sizeLog = logs.size();
  viewer.indexLog = startIndex;
  while (indexLog < logs.size() && viewer.update()) {
    // Import LogIndex from viewer
    indexLog = viewer.indexLog;

    //Assign DOF
    motorsDOF.assignOp(logs[indexLog], "pos", "");
    model.setDOF(motorsDOF, true);

    Leph::VectorLabel pressures = logs[indexLog].extract("pressure").rename("pressure", "");
    pressures = pressures.renameLabels(":val:","_gauge_");

    model.updatePressure(pressures);

    model.updateBase();

    viewer.drawFrame(model.centerOfMass("origin"),
                     model.getCOMBasisOrientation(), 0.2);

    // Print pressures
    for (const auto& pEntry : model.getPressureValues()) {
      double halfZ = pEntry.second / 10000;
      Eigen::Vector3d gaugePos = model.position(pEntry.first, "origin");
      Eigen::Vector3d halfSize = Eigen::Vector3d(0.005, 0.005, halfZ);
      viewer.drawBox(halfSize,
                     gaugePos + halfSize,
                     Eigen::Matrix3d::Identity(),
                     1.0, 0.0, 0.0);
    }

    std::string basisUsed("origin");
    Eigen::Vector3d projectedCOP = model.getCOP(basisUsed);
    projectedCOP.z() = 0;

    Eigen::Vector3d projectedCOM = model.centerOfMass(basisUsed);
    projectedCOM.z() = 0;

    // Display trajectories
    viewer.addTrackedPoint(projectedCOM,
                           Leph::ModelViewer::Yellow);
    viewer.addTrackedPoint(model.position("left_arch_center", "origin"), 
                           Leph::ModelViewer::Red);
    viewer.addTrackedPoint(model.position("right_arch_center", "origin"), 
                           Leph::ModelViewer::Green);
    viewer.addTrackedPoint(projectedCOP, 
                           Leph::ModelViewer::Blue);
        
    //Display models
    Leph::ModelDraw(model, viewer);
  }
  return 0;
}

