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
    motors->get("Pied G")->setAngle(outputs("left foot roll"));
    motors->get("Cheville G")->setAngle(outputs("left foot pitch"));
    motors->get("Genou G")->setAngle(outputs("left knee"));
    motors->get("Cuisse G")->setAngle(outputs("left hip pitch"));
    motors->get("Hanche G Lat")->setAngle(outputs("left hip roll"));
    motors->get("Hanche G Rot")->setAngle(outputs("left hip yaw"));
    motors->get("Pied D")->setAngle(outputs("right foot roll"));
    motors->get("Cheville D")->setAngle(outputs("right foot pitch"));
    motors->get("Genou D")->setAngle(outputs("right knee"));
    motors->get("Cuisse D")->setAngle(outputs("right hip pitch"));
    motors->get("Hanche D Lat")->setAngle(outputs("right hip roll"));
    motors->get("Hanche D Rot")->setAngle(outputs("right hip yaw"));
}

Leph::VectorLabel retrieveMotorsAngle(Rhoban::Motors* motors)
{
    return Leph::VectorLabel(
        "left foot roll", motors->get("Pied G")->getRelAngle(),
        "left foot pitch", motors->get("Cheville G")->getRelAngle(),
        "left knee", motors->get("Genou G")->getRelAngle(),
        "left hip pitch", motors->get("Cuisse G")->getRelAngle(),
        "left hip roll", motors->get("Hanche G Lat")->getRelAngle(),
        "left hip yaw", motors->get("Hanche G Rot")->getRelAngle(),
        "right foot roll", motors->get("Pied D")->getRelAngle(),
        "right foot pitch", motors->get("Cheville D")->getRelAngle(),
        "right knee", motors->get("Genou D")->getRelAngle(),
        "right hip pitch", motors->get("Cuisse D")->getRelAngle(),
        "right hip roll", motors->get("Hanche D Lat")->getRelAngle(),
        "right hip yaw", motors->get("Hanche D Rot")->getRelAngle());
}

void runWalk(Rhoban::Motors* motors, Leph::CartWalkProxy& walk, 
    const Leph::VectorLabel& staticParams, const Leph::VectorLabel& dynamicParams, 
    double duration)
{
    const double freq = 50.0;
    for (double t=0.0;t<=duration;t+=1.0/freq) {
        walk.exec(1.0/freq, dynamicParams, staticParams);
        Leph::VectorLabel outputs = walk.lastOutputs();
        forwardMotorsOrders(motors, outputs);
        std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
    }
}

int main()
{
    Leph::CartWalkProxy walk;
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();
    std::cout << (staticParams+dynamicParams) << std::endl;
    
    try {
        std::cout << "Connection..." << std::endl;
        Rhoban::Robot robot(new CommandsStore, "local");
        robot.connect("127.0.0.1", 7777);
        if (!robot.isConnected()) {
            std::cout << "No connection" << std::endl;
            return 0;
        }
        robot.loadEnvironment("/home/rhoban/Environments/RhobanServer/Mowgly");
        Rhoban::Motors* motors = robot.getMotors();
        std::cout << "Starting motors dispatcher" << std::endl;
        motors->start(100);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        /*
        std::cout << "Walk enable=0" << std::endl;
        dynamicParams("enabled") = 0;
        staticParams("zOffset") = 3;
        staticParams("riseGain") = 4;
        runWalk(motors, walk, staticParams, dynamicParams, 3.0);
        dynamicParams("enabled") = 1;
        dynamicParams("step") = 6;
        runWalk(motors, walk, staticParams, dynamicParams, 6.0);
        dynamicParams("enabled") = 0;
        runWalk(motors, walk, staticParams, dynamicParams, 3.0);
        */

        const double freq = 5.0;
        for (double t=0.0;t<=60.0;t+=1.0/freq) {
            std::cout << retrieveMotorsAngle(motors) << std::endl;
            std::this_thread::sleep_for(
                    std::chrono::milliseconds((int)(1000/freq)));
        }
        
        /*
        std::cout << "Walk enable=1" << std::endl;
        dynamicParams("enabled") = 1;
        runWalk(motors, walk, staticParams, dynamicParams, 5.0);
        std::cout << "Walk enable=0" << std::endl;
        dynamicParams("enabled") = 0;
        runWalk(motors, walk, staticParams, dynamicParams, 5.0);
        */

        std::cout << "Stopping" << std::endl;
        motors->stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } catch (std::string err) {
        std::cout << "Exception : " << err << std::endl;
    }

    return 0;
}

