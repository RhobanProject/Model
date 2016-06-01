#include <iostream>
#include "Utils/Differentiation.hpp"
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::ModelViewer viewer(1200, 900);
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    model.updateBase();

    //Load logged data
    Leph::MatrixLabel logs;
    logs.load(
        //"../../These/Data/logs-2015-05-16/model_2015-05-16-18-47-31.log");
        "../../These/Data/logs-2015-05-16/model_2015-05-16-18-49-57.log");

    //Initializing DOF rolling buffer
    Leph::Differentiation buffer(20, 4);
    
    Leph::Plot plot;
    Leph::Scheduling scheduling(50.0);
    size_t stepCount = 1;
    size_t index = 0;
    size_t count = 0;
    double t = 0.0;
    while (viewer.update()) {
        //Import motors orders
        Leph::VectorLabel pos = 
            logs[index].extract("motor").rename("motor", "");
        //Appending to rolling buffer
        if (count%stepCount == 0) {
            buffer.add(t, pos);
        }
        //Compute smooth position, velocity and acceleration
        Leph::VectorLabel vel = pos;
        Leph::VectorLabel acc = pos;
        if (buffer.isFull()) {
            double middleT = (buffer.minTime()+buffer.maxTime())/2.0;
            pos = buffer.position(middleT);
            vel = buffer.velocity(middleT);
            acc = buffer.acceleration(middleT);
            pos.setOrAppend("time:time", middleT);
            vel.setOrAppend("time:time", middleT);
            acc.setOrAppend("time:time", middleT);
        }
        plot.add(pos.rename("", "pos"));
        plot.add(vel.rename("", "vel"));
        plot.add(acc.rename("", "acc"));
        //Inject DOF into the model
        model.get().setDOF(pos, false);
        model.setOrientation( 
            Eigen::AngleAxisd(-logs[index]("sensor:roll"), 
                Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(logs[index]("sensor:pitch"), 
                Eigen::Vector3d::UnitY()).toRotationMatrix(),
            false);
        //Contraint the model on the ground
        model.updateBase();
        //Compute ZMP point
        Eigen::Vector3d zmp = model.zeroMomentPoint(
            "origin", vel, acc);
        std::cout << t << " " << zmp.x() << " " << zmp.y() << std::endl;
        //Track somes points
        viewer.addTrackedPoint(
            model.get().position("trunk", "origin"), 
            Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            model.get().centerOfMass("origin"), 
            Leph::ModelViewer::Blue);
        viewer.addTrackedPoint(
            zmp, 
            Leph::ModelViewer::Red);
        //Display model
        Leph::ModelDraw(model.get(), viewer);
        //Waiting
        scheduling.wait();
        count++;
        if (count%stepCount == 0 && index < logs.size()-1) {
            index++;
            t += 0.02;
        }
    }
    plot
        .plot("time:time", "pos:left_knee")
        .plot("time:time", "vel:left_knee")
        .plot("time:time", "acc:left_knee")
        .render();
    
    return 0;
}

