#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedPressureModel.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

#include "Model/HumanoidModelWithToe.hpp"
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
    Leph::HumanoidModelWithToe model(rbdlModel);

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

        const std::vector<std::string> plates = {"LeftBase", "RightBase", "LeftToe", "RightToe"};
        const std::vector<std::string> frames = {"left_arch", "right_arch", "left_toe", "right_toe"};
        //TODO set pressure information and use them
        for (unsigned int plateID = 0; plateID < plates.size(); plateID++) {
          std::string plate = plates[plateID];
          std::string frame = frames[plateID];
          for (int gaugeID = 0; gaugeID < 4; gaugeID++) {
            std::ostringstream logName, frameName;
            logName << plate << ":val:" << gaugeID;
            frameName << frame << "_gauge_" << gaugeID;
            double val = logs[indexLog](logName.str());
            Eigen::Vector3d gaugePos = model.position(frameName.str(), "origin");
            Eigen::Vector3d halfSize = Eigen::Vector3d(0.005, 0.005, val / 10000);
              viewer.drawBox(halfSize,
                           gaugePos + halfSize,
                           Eigen::Matrix3d::Identity(),
                           1.0, 0.0, 0.0);
          }
        }

        // Display trajectories
        Eigen::Vector3d projectedCoM = model.centerOfMass("origin");
        projectedCoM.z() = -0.667;
        viewer.addTrackedPoint(projectedCoM,
                               Leph::ModelViewer::Yellow);
        viewer.addTrackedPoint(model.position("left_arch_tip", "origin"), 
                               Leph::ModelViewer::Red);
        viewer.addTrackedPoint(model.position("right_arch_tip", "origin"), 
                               Leph::ModelViewer::Green);
        
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

