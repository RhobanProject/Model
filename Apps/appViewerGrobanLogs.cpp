#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/PressureModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"

#include "Model/ModelBuilder.hpp"

using namespace Leph;

int main(int argc, char** argv)
{
    //Command line arguments
    if (argc != 2) {
        std::cout << "Usage: ./app logsFile" << std::endl;
        return -1;
    }

    std::string logsFile = argv[1];
    std::cout << "Loading " << logsFile << std::endl;

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

    //Ploting curves
    //std::cout << "Plotting references and motors" << std::endl;
    //logs.plot()
    //    .plot("time:timestamp", "goal:*")
    //    .plot("time:timestamp", "pos:*")
    //    .render();
    //std::cout << "Plotting sensors" << std::endl;
    //logs.plot()
    //    .plot("time:timestamp", "sensor:*")
    //    .render();
    //std::cout << "Plotting pressure" << std::endl;
    //logs.plot()
    //    .plot("index", "pressure:*")
    //    .render();
    //std::cout << "Plotting timming" << std::endl;
    //logs.plot()
    //    .plot("index", "time:*")
    //    .render();
    //std::cout << "Plotting walk parameters" << std::endl;
    //logs.plot()
    //    .plot("index", "walk:*")
    //    .render();
    
    //Initialize model instances
    RBDL::Model rbdlModel = generateGrobanWithToe(true);
    Leph::PressureModel model(rbdlModel);

    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling;

    //Initialize DOF vector
    Leph::VectorLabel outputsDOF = logs[0].extract("goal").rename("goal", "");
    Leph::VectorLabel motorsDOF  = logs[0].extract("pos").rename("pos", "");
            
    //Main loop
    double freq = 50.0;
    scheduling.setFrequency(freq);
    double t = logs[0]("time:timestamp");
    size_t indexLog = 0;
    bool isPaused = false;
    int viewMode = 0;
    while (viewer.update()) {
        //Find current log index (associated with time)
        indexLog = 0;
        while (
            indexLog < logs.size()-1 && 
            logs[indexLog]("time:timestamp") < t
        ) {
            indexLog++;
        }
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
        //Assign DOF
        outputsDOF.assignOp(logs[indexLog], "goal", "");
        motorsDOF.assignOp(logs[indexLog], "pos", "");
        model.setDOF(motorsDOF, true);

        //TODO set pressure information and use them
        Leph::VectorLabel pressures = logs[indexLog].extract("pressure").rename("pressure", "");
        pressures = pressures.renameLabels(":val:","_gauge_");

        model.updatePressure(pressures);

        model.updateBase();

        for (const auto& pEntry : model.getPressureValues()) {
          double halfZ = pEntry.second / 10000;
          Eigen::Vector3d gaugePos = model.position(pEntry.first, "origin");
          Eigen::Vector3d halfSize = Eigen::Vector3d(0.005, 0.005, halfZ);
          viewer.drawBox(halfSize,
                         gaugePos + halfSize,
                         Eigen::Matrix3d::Identity(),
                         1.0, 0.0, 0.0);
        }

        Eigen::Vector3d projectedCOP = model.getCOP("origin");
        projectedCOP.z() = 0;

        viewer.drawBox(Eigen::Vector3d(0.005, 0.005,0.5) ,
                       projectedCOP,
                       Eigen::Matrix3d::Identity(),
                       0.0, 0.0, 1.0);

        // Display trajectories
        Eigen::Vector3d projectedCoM = model.centerOfMass("origin");
        projectedCoM.z() = 0;
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
        //Phase cycling
        std::cout << "t= " << t << " index=" 
            << indexLog << "/" << logs.size()-1 << std::endl;
        if (!isPaused) {
            t += 1000.0/freq;
        }
        if (t > logs[logs.size()-1]("time:timestamp") + 10000.0) {
            t = logs[0]("time:timestamp");
            indexLog = 0;
        }
    }

    return 0;
}

