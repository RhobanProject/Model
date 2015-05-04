#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

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
    std::cout << "Plotting references and motors" << std::endl;
    logs.plot()
        .plot("time:timestamp", "output:*")
        .plot("time:timestamp", "motor:*")
        .render();
    std::cout << "Plotting sensors" << std::endl;
    logs.plot()
        .plot("time:timestamp", "sensor:*")
        .render();
    std::cout << "Plotting timming" << std::endl;
    logs.plot()
        .plot("index", "time:*")
        .render();
    
    //Initialize model instances
    Leph::HumanoidFixedModel modelOutputs("../../Data/sigmaban.urdf");
    Leph::HumanoidFixedModel modelMotors("../../Data/sigmaban.urdf");
    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling;

    //Initialize DOF vector
    Leph::VectorLabel outputsDOF = 
        logs[0].extract("output").rename("output", "");
    Leph::VectorLabel motorsDOF = 
        logs[0].extract("motor").rename("motor", "");
            
    //Main loop
    double freq = 50.0;
    scheduling.setFrequency(freq);
    double t = logs[0]("time:timestamp")/1000.0;
    size_t indexLog = 0;
    while (viewer.update()) {
        while (
            indexLog < logs.size()-1 && 
            logs[indexLog]("time:timestamp")/1000.0 < t
        ) {
            indexLog++;
        }
        std::cout << "t= " << t << " index=" 
            << indexLog << "/" << logs.size()-1 << std::endl;
        //Assign DOF
        outputsDOF.assignOp(logs[indexLog], "output", "");
        motorsDOF.assignOp(logs[indexLog], "motor", "");
        outputsDOF.mulOp(M_PI/180.0);
        motorsDOF.mulOp(M_PI/180.0);
        modelOutputs.get().setDOF(outputsDOF);
        modelMotors.get().setDOF(motorsDOF);
        //Contraint the model on the ground, integrate movement
        modelOutputs.updateBase();
        modelMotors.updateBase();
        //Set IMU data for motors real model state
        modelMotors.setOrientation(
            logs[indexLog]("sensor:Pitch")*M_PI/180.0, 
            logs[indexLog]("sensor:Roll")*M_PI/180.0);
        
        //Display captured center of mass ground projection
        Eigen::Vector3d com = modelMotors.get().centerOfMass("origin");
        com.z() = 0.0;
        viewer.addTrackedPoint(com);
        
        //Display models
        Leph::ModelDraw(modelOutputs.get(), viewer);
        Leph::ModelDraw(modelMotors.get(), viewer);
        //Waiting
        scheduling.wait();
        //Phase cycling
        t += 1.0/freq;
        if (t > logs[logs.size()-1]("time:timestamp")/1000.0 + 2.0) {
            t = logs[0]("time:timestamp")/1000.0;
            indexLog = 0;
        }
    }

    return 0;
}

