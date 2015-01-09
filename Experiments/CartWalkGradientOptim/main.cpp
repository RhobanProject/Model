#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

//SDK
#include "main/Command.h"
#include "robot/Robot.h"
#include "robot/Robots.h"

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

void forwardMotorsOrders(Rhoban::Motors* motors, const Leph::VectorLabel& outputs)
{
    std::cout << outputs("left foot roll") << " " << outputs("right foot roll") << std::endl;
    motors->get("Pied G")->setRelAngle(outputs("left foot roll"));
    motors->get("Cheville G")->setRelAngle(outputs("left foot pitch"));
    motors->get("Genou G")->setRelAngle(outputs("left knee"));
    motors->get("Cuisse G")->setRelAngle(outputs("left hip pitch"));
    motors->get("Hanche G Lat")->setRelAngle(outputs("left hip roll"));
    motors->get("Hanche G Rot")->setRelAngle(outputs("left hip yaw"));
    motors->get("Pied D")->setRelAngle(outputs("right foot roll"));
    motors->get("Cheville D")->setRelAngle(outputs("right foot pitch"));
    motors->get("Genou D")->setRelAngle(outputs("right knee"));
    motors->get("Cuisse D")->setRelAngle(outputs("right hip pitch"));
    motors->get("Hanche D Lat")->setRelAngle(outputs("right hip roll"));
    motors->get("Hanche D Rot")->setRelAngle(outputs("right hip yaw"));
}

int main()
{
    Leph::CartWalkProxy walk;
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();
    
    dynamicParams("enabled") = 0;
    
    try {
        Rhoban::Robot robot(new CommandsStore, "local");
        robot.connect("127.0.0.1", 7777);
        if (!robot.isConnected()) {
            std::cout << "No connection" << std::endl;
            return 0;
        }
        robot.loadEnvironment("/home/rhoban/Environments/RobotBoard/Mowgly/");
        Rhoban::Motors* motors = robot.getMotors();
        motors->start(100);

        //std::this_thread::sleep_for(std::chrono::seconds(1));
        //motors->goToInit(2.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        double freq = 50.0;
        for (double t=0.0;t<=5.0;t+=1.0/freq) {
            walk.exec(1.0/freq, dynamicParams, staticParams);
            Leph::VectorLabel outputs = walk.lastOutputs();
            std::cout << outputs << std::endl;
            forwardMotorsOrders(motors, outputs);
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
        }
        dynamicParams("enabled") = 1;
        for (double t=0.0;t<=5.0;t+=1.0/freq) {
            walk.exec(1.0/freq, dynamicParams, staticParams);
            Leph::VectorLabel outputs = walk.lastOutputs();
            std::cout << outputs << std::endl;
            forwardMotorsOrders(motors, outputs);
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
        }
        dynamicParams("enabled") = 0;
        for (double t=0.0;t<=2.0;t+=1.0/freq) {
            walk.exec(1.0/freq, dynamicParams, staticParams);
            Leph::VectorLabel outputs = walk.lastOutputs();
            std::cout << outputs << std::endl;
            forwardMotorsOrders(motors, outputs);
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
        }

        //motors->goToInit(1.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        motors->stop();
    } catch (std::string err) {
        std::cout << "Exception : " << err << std::endl;
    }

    return 0;
}

