#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedPressureModel.hpp"
#include "IKWalk/IKWalk.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

int main(int argc, char** argv)
{
    //Command line arguments
    if (argc != 3) {
        std::cout << "Usage: ./app [sigmaban|grosban] logsFile" << std::endl;
        return -1;
    }

    Leph::RobotType type;
    if (std::string(argv[1]) == "sigmaban") {
        type = Leph::SigmabanModel;
    } else if (std::string(argv[1]) == "grosban") {
        type = Leph::GrosbanModel;
    } else {
        std::cout << "Usage: ./app [sigmaban|grosban] logsFile" << std::endl;
        return -1;
    }
    std::string logsFile = argv[2];
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
    std::cout << "Plotting references and motors" << std::endl;
    logs.plot()
        .plot("time:timestamp", "goal:*")
        .plot("time:timestamp", "pos:*")
        .render();
    std::cout << "Plotting sensors" << std::endl;
    logs.plot()
        .plot("time:timestamp", "sensor:*")
        .render();
    std::cout << "Plotting pressure" << std::endl;
    logs.plot()
        .plot("index", "pressure:*")
        .render();
    std::cout << "Plotting timming" << std::endl;
    logs.plot()
        .plot("index", "time:*")
        .render();
    std::cout << "Plotting walk parameters" << std::endl;
    logs.plot()
        .plot("index", "walk:*")
        .render();
    
    //Initialize model instances
    Leph::HumanoidFixedModel modelOutputs(type);
    Leph::HumanoidFixedPressureModel modelMotors(type);
    Leph::HumanoidFixedModel modelWalk(type);
    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling;

    //Initialize DOF vector
    Leph::VectorLabel outputsDOF = 
        logs[0].extract("goal").rename("goal", "");
    Leph::VectorLabel motorsDOF = 
        logs[0].extract("pos").rename("pos", "");
    Leph::VectorLabel walkParams = 
        logs[0].extract("walk");
            
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
        modelOutputs.get().setDOF(outputsDOF, false);
        modelMotors.get().setDOF(motorsDOF, false);
        //Walk model
        walkParams.assignOp(logs[indexLog], "walk");
        Leph::IKWalk::Parameters params = 
            Leph::IKWalk::convertVectorLabel(walkParams);
        bool success = Leph::IKWalk::walk(
            modelWalk.get(), params, logs[indexLog]("time:phase"), 0.02);
        if (!success) {
            std::cout << "IKWalk error" << std::endl;
            return -1;
        }
        //Assiging pressure
        //XXX TODO Using old sigmaban pressure convention
        //=> To be updated
        modelMotors.setPressure(
            logs[indexLog]("pressure:weight"),
            logs[indexLog]("pressure:left_weight")/1000.0
                /logs[indexLog]("pressure:weight"),
            logs[indexLog]("pressure:right_weight")/1000.0
                /logs[indexLog]("pressure:weight"),
            logs[indexLog]("pressure:left_x")/100.0,
            logs[indexLog]("pressure:left_y")/100.0,
            logs[indexLog]("pressure:right_x")/100.0,
            logs[indexLog]("pressure:right_y")/100.0);
        //Contraint the model on the ground, integrate movement
        modelOutputs.updateBase();
        modelMotors.updateBase();
        modelWalk.updateBase();
        //Set IMU data for motors real model state
        modelMotors.setOrientation(
            logs[indexLog]("sensor:pitch"), 
            logs[indexLog]("sensor:roll"));
        modelMotors.setYaw(modelMotors.getSupportFoot(), 
            logs[indexLog]("sensor:gyro_yaw")*3.14/180.0
            -logs[0]("sensor:gyro_yaw")*3.14/180.0); //TODO DEG2RAD ??? Inside model log
        
        //Display captured center of mass ground projection
        if (viewMode == 1) {
        viewer.addTrackedPoint(
            modelOutputs.get().centerOfMass("origin"), 
            Leph::ModelViewer::Yellow);
        viewer.addTrackedPoint(
            modelOutputs.get().position("left_foot_tip", "origin"), 
            Leph::ModelViewer::Red);
        viewer.addTrackedPoint(
            modelOutputs.get().position("right_foot_tip", "origin"), 
            Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            modelOutputs.get().position("trunk", "origin"), 
            Leph::ModelViewer::Blue);
        viewer.addTrackedPoint(
            modelOutputs.get().position("camera", "origin"), 
            Leph::ModelViewer::Cyan);
        }
        if (viewMode == 2) {
        viewer.addTrackedPoint(
            modelMotors.get().centerOfMass("origin"), 
            Leph::ModelViewer::Yellow);
        viewer.addTrackedPoint(
            modelMotors.get().position("left_foot_tip", "origin"), 
            Leph::ModelViewer::Red);
        viewer.addTrackedPoint(
            modelMotors.get().position("right_foot_tip", "origin"), 
            Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            modelMotors.get().position("trunk", "origin"), 
            Leph::ModelViewer::Blue);
        viewer.addTrackedPoint(
            modelMotors.get().position("camera", "origin"), 
            Leph::ModelViewer::Cyan);
        }
        
        //Display models
        if (viewMode == 0) {
            Leph::ModelDraw(modelOutputs.get(), viewer);
            Leph::ModelDraw(modelMotors.get(), viewer);
        } 
        if (viewMode == 1) {
            Leph::ModelDraw(modelOutputs.get(), viewer);
        } 
        if (viewMode == 2) {
            Leph::ModelDraw(modelMotors.get(), viewer);
        } 
        if (viewMode == 3) {
            Leph::ModelDraw(modelWalk.get(), viewer);
        } 
        //Waiting
        scheduling.wait();
        //Phase cycling
        std::cout << "t= " << t << " index=" 
            << indexLog << "/" << logs.size()-1 << std::endl;
        if (!isPaused) {
            t += 1000.0/freq;
        }
        if (t > logs[logs.size()-1]("time:timestamp") + 2000.0) {
            t = logs[0]("time:timestamp");
            indexLog = 0;
        }
    }

    return 0;
}

