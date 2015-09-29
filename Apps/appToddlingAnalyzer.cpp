#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/PressureModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"

#include "Model/ModelBuilder.hpp"
#include "Utils/Chrono.hpp"

//#define VIEWER true

using namespace Leph;

int main(int argc, char** argv)
{
    //Command line arguments
    if (argc != 3) {
        std::cout << "Usage: ./app <logsFile> <dst>" << std::endl;
        return -1;
    }

    std::string logsFile = argv[1];
    std::cerr << "Loading " << logsFile << std::endl;

    //Loading data
    Leph::MatrixLabel logs, mdpEntries;
    logs.load(logsFile);

    //Print data informations
    std::cerr << "Loaded " 
        << logs.size() << " points with " 
        << logs.dimension() << " entries" << std::endl;
    if (logs.size() == 0) {
        return 0;
    }
    
    double filteredDiffY = 0;
    double disc = 0.95;

    //Initialize model instances
    Leph::PressureModel model(generateGrobanWithToe(true));


    //Initialize DOF vector
    Leph::VectorLabel outputsDOF = logs[0].extract("goal").rename("goal", "");
    Leph::VectorLabel motorsDOF  = logs[0].extract("pos").rename("pos", "");

    Eigen::Vector3d lastMeasuredCOM, lastMeasuredCOP;
            
    //Main loop
    size_t indexLog = 0;
    bool isPaused = false;
    Leph::Chrono chrono;
#if VIEWER
    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling;
    scheduling.setFrequency(freq);
    int viewMode = 0;
    while (indexLog < logs.size() && viewer.update()) {
#else
      while(indexLog < logs.size()) {
#endif
#if VIEWER
        //Interface control
        if (viewer.isKeyPressed(sf::Keyboard::I)) {
            t = logs[0]("time:timestamp");
            indexLog = 0;
        }
        if (viewer.isKeyPressed(sf::Keyboard::P)) {
            isPaused = true;
        }
        if (viewer.isKeyPressed(sf::Keyboard::C)) {
            isPaused = false;
        }
        if (viewer.isKeyPressed(sf::Keyboard::PageUp)) {
            t += 1000.0;
        }
        if (viewer.isKeyPressed(sf::Keyboard::PageDown)) {
            t -= 1000.0;
        }
        if (viewer.isKeyPressed(sf::Keyboard::M)) {
            viewMode++;
            if (viewMode > 3) {
                viewMode = 0;
            } 
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
#endif
        //Assign DOF
        outputsDOF.assignOp(logs[indexLog], "goal", "");
        motorsDOF.assignOp(logs[indexLog], "pos", "");
        model.setDOF(motorsDOF, true);

        //TODO set pressure information and use them
        Leph::VectorLabel pressures = logs[indexLog].extract("pressure").rename("pressure", "");
        pressures = pressures.renameLabels(":val:","_gauge_");

        model.updatePressure(pressures);

        chrono.start("UpdateBase");
        model.updateBase();
        chrono.stop("UpdateBase");
        //chrono.print();

#if VIEWER
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
#endif

        Eigen::Vector3d projectedCOP = model.getCOP("right_arch_center");
        projectedCOP.z() = 0;

        // Display trajectories
        Eigen::Vector3d projectedCoM = model.centerOfMass("right_arch_center");
        projectedCoM.z() = 0;

        double diffY = projectedCoM.y() - projectedCOP.y();
        filteredDiffY = filteredDiffY * disc + diffY * (1 - disc);

        if (indexLog > 0) {
          VectorLabel data = logs[indexLog];
          data.subOp(logs[indexLog -1]);

          VectorLabel posData = data.extract("pos");
          double squaredDiff(0);
          for (size_t i = 0; i < posData.size(); i++) {
            double diff = posData(i);
            squaredDiff += diff * diff;
          }

          data.append("diffPos", std::sqrt(squaredDiff));

          //data.append("srcPhase", logs[indexLog-1]("phase"));
          //data.append("srcCOPY", lastMeasuredCOP.y());
          //data.append("srcCOMY", lastMeasuredCOM.y());
          //data.append("targetY", logs[indexLog-1]("targetComY"));
          //data.append("nextPhase", logs[indexLog]("phase"));
          //data.append("nextCOPY", projectedCOP.y());
          //data.append("nextCOMY", projectedCoM.y());
          mdpEntries.append(data);
        }
        lastMeasuredCOP = projectedCOP;
        lastMeasuredCOM = projectedCoM;

#if VIEWER
        viewer.addTrackedPoint(projectedCoM,
                               Leph::ModelViewer::Yellow);
        viewer.addTrackedPoint(model.position("left_arch_center", "origin"), 
                               Leph::ModelViewer::Red);
        viewer.addTrackedPoint(model.position("right_arch_center", "origin"), 
                               Leph::ModelViewer::Green);
        viewer.addTrackedPoint(projectedCOP, 
                               Leph::ModelViewer::Blue);
        
        //Display models
        Leph::ModelDraw(model, viewer);
        //Waiting
        scheduling.wait();
#endif
        if (!isPaused) {
          indexLog++;
        }
    }

    std::string dstFile = argv[2];
    mdpEntries.save(dstFile);

    return 0;
}

