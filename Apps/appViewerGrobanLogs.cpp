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
    if (argc != 2) {
        std::cout << "Usage: ./app logsFile" << std::endl;
        return -1;
    }

    std::string logsFile = argv[1];
    std::cerr << "Loading " << logsFile << std::endl;

    //Loading data
    Leph::MatrixLabel logs;
    logs.load(logsFile);

    //Print data informations
    std::cerr << "Loaded " 
        << logs.size() << " points with " 
        << logs.dimension() << " entries" << std::endl;
    if (logs.size() == 0) {
        return 0;
    }

    std::cout << "timestamp,comX,comY,copX,copY,diffY,filteredDiffY" << std::endl;
    
    double filteredDiffY = 0;
    double disc = 0.95;

    //Initialize model instances
    Leph::PressureModel model(generateGrobanWithToe(true));


    //Initialize DOF vector
    Leph::VectorLabel outputsDOF = logs[0].extract("goal").rename("goal", "");
    Leph::VectorLabel motorsDOF  = logs[0].extract("pos").rename("pos", "");
            
    //Main loop
    double t = logs[0]("time:timestamp");
    double freq = 50.0;
    size_t indexLog = 0;
    bool isPaused = false;
    Leph::Chrono chrono;
#if VIEWER
    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling;
    scheduling.setFrequency(freq);
    int viewMode = 0;
    while (viewer.update()) {
#else
    while(true) {
#endif
        //Find current log index (associated with time)
        indexLog = 0;
        while (
            indexLog < logs.size()-1 && 
            logs[indexLog]("time:timestamp") < t
        ) {
            indexLog++;
        }
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

        std::cout << t << ','
                  << projectedCoM.x() << "," << projectedCoM.y() << ","
                  << projectedCOP.x() << "," << projectedCOP.y() << ","
                  << diffY << "," << filteredDiffY
                  << std::endl;

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
        ////Phase cycling
        //std::cout << "t= " << t << " index=" 
        //    << indexLog << "/" << logs.size()-1 << std::endl;
        if (!isPaused) {
            t += 1000.0/freq;
        }
        if (t > logs[logs.size()-1]("time:timestamp")){
          break;
            t = logs[0]("time:timestamp");
            indexLog = 0;
        }
    }

    return 0;
}

