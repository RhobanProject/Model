#include <iostream>
#include "Types/MatrixLabel.hpp"
#include "Utils/Differentiation.hpp"
#include "Model/HumanoidFixedModel.hpp"
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

    //Pärsing robot model type
    Leph::RobotType type;
    if (std::string(argv[1]) == "sigmaban") {
        type = Leph::SigmabanModel;
    } else if (std::string(argv[1]) == "grosban") {
        type = Leph::GrosbanModel;
    } else {
        std::cout << "Usage: ./app [sigmaban|grosban] logsFile" << std::endl;
        return -1;
    }

    //Loading data
    std::string logsFile = argv[2];
    std::cout << "Loading " << logsFile << std::endl;
    Leph::MatrixLabel logs;
    logs.load(logsFile);

    //Print data informations
    std::cout << "Loaded " 
        << logs.size() << " points with " 
        << logs.dimension() << " entries" << std::endl;
    if (logs.size() == 0) {
        return 0;
    }

    //Plotting
    std::cout << "Plotting references and motors" << std::endl;
    logs.plot()
        .plot("time:timestamp", "output:*")
        .plot("time:timestamp", "motor:*")
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
    Leph::HumanoidFixedModel modelMotors(type);
    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling;

    //Initialize DOF vector
    Leph::VectorLabel pos = 
        logs[0].extract("motor").rename("motor", "");

    //Initializing DOF rolling buffer
    //for differentiation
    Leph::Differentiation buffer(30, 3);
            
    Leph::Plot plot;
    //Main loop
    scheduling.setFrequency(50.0);
    size_t indexLog = 0;
    while (viewer.update()) {
        //Retrieve read DOF
        pos.assignOp(logs[indexLog], "motor", "");
        //Appending to rolling buffer
        buffer.add(logs[indexLog]("time:timestamp")/1000.0, pos);
        //Compute smooth position, velocity and acceleration
        Leph::VectorLabel vel = pos;
        Leph::VectorLabel acc = pos;
        if (buffer.isFull()) {
            double middleT = logs[indexLog]("time:timestamp")/1000.0;
            //XXX double middleT = (buffer.minTime()+buffer.maxTime())/2.0;
            pos = buffer.position(middleT);
            vel = buffer.velocity(middleT);
            acc = buffer.acceleration(middleT);
        }
        //Assigning to model
        modelMotors.get().setDOF(pos, false);
        //Set IMU data for motors real model state
        modelMotors.setOrientation(
            logs[indexLog]("sensor:pitch"), 
            logs[indexLog]("sensor:roll"));
        //Contraint the model on the ground, integrate movement
        modelMotors.updateBase();
        //Compute ZMP point
        Eigen::Vector3d zmp = modelMotors.zeroMomentPoint(
            "origin", vel, acc);
        //Display captured center of mass ground projection
        double wLeft = logs[indexLog]("pressure:w1")/(1000.0*logs[indexLog]("pressure:w"));
        double wRight = logs[indexLog]("pressure:w2")/(1000.0*logs[indexLog]("pressure:w"));
        Eigen::Vector3d rLeft(logs[indexLog]("pressure:x1"), logs[indexLog]("pressure:y1"), 0.0);
        Eigen::Vector3d rRight(logs[indexLog]("pressure:x2"), logs[indexLog]("pressure:y2"), 0.0);
        Eigen::Vector3d pLeft = rLeft + modelMotors.get().position("left_foot_tip", "origin");
        Eigen::Vector3d pRight = rRight + modelMotors.get().position("right_foot_tip", "origin");
        Eigen::Vector3d cop = (wLeft*pLeft + wRight*pRight)/(wLeft+wRight);
        cop.z() = 0.0;
        //Adding to plot
        plot.add(Leph::VectorLabel(
            "t", logs[indexLog]("time:timestamp"),
            "cop_x", cop.x(),
            "cop_y", cop.y(),
            "zmp_x", zmp.x(),
            "zmp_y", zmp.y()
        ));
        //Display model
        Leph::ModelDraw(modelMotors.get(), viewer);
        viewer.addTrackedPoint(
            cop, Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            zmp, Leph::ModelViewer::Red);
        //Waiting
        scheduling.wait();
        //Phase cycling
        std::cout << "t= " << logs[indexLog]("time:timestamp") << " index=" 
            << indexLog << "/" << logs.size()-1 << std::endl;
        if (indexLog < logs.size()-1) {
            indexLog++;
        } else {
            indexLog = 0;
        }
    }
    plot
        .plot("t", "all")
        .render();
    plot
        .plot("cop_x", "zmp_x", Leph::Plot::Points, "t")
        .render();
    plot
        .plot("cop_y", "zmp_y", Leph::Plot::Points, "t")
        .render();

    return 0;
}

