#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <signal.h>
#include <unistd.h>

//SDK
#include "main/Command.h"
#include "robot/Robot.h"
#include "robot/Robots.h"

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

//UTILS
#include "MotionCapture.hpp"

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

bool askedExit = false;

void signal_handler(int s)
{
    askedExit = true;
}

int main()
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signal_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    Leph::CartWalkProxy walk;
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();
    std::cout << (staticParams+dynamicParams) << std::endl;

    Rhoban::MotionCapture motionCapture;
    motionCapture.setCaptureStream("tcp://192.168.16.10:3232");
    //motionCapture.averageCoefPos = 0.9;
    //motionCapture.maxInvalidTick = 20;

    const double forwardReference = -0.0939998;
    const double turnReference = 0.0;
    const double lateralReference = 0.145968;
    const double freq = 50.0;
    
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
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        staticParams("zOffset") = 3;
        staticParams("riseGain") = 5;
        staticParams("hipOffset") = 13;

        std::cout << "Walk disabled" << std::endl;
        dynamicParams("enabled") = 0;
        runWalk(motors, walk, staticParams, dynamicParams, 2.0);
        std::cout << "Walk enabled" << std::endl;
        dynamicParams("enabled") = 1;
        double deltaStep = 0.0;
        double deltaLateral = 0.0;
        double deltaTurn = 0.0;
        double discountCoef = 0.85;
        while (!askedExit) {
            motionCapture.tick(1.0/freq);
            bool isValid = motionCapture.pos.isValid;
            double forwardError = motionCapture.pos.z - forwardReference;
            double turnError = motionCapture.pos.azimuth - turnReference;
            double lateralError = motionCapture.pos.x - lateralReference;

            if (isValid) {
                deltaStep = discountCoef*deltaStep + (1.0-discountCoef)*10.0*forwardError;
                deltaLateral = discountCoef*deltaLateral + (1.0-discountCoef)*50.0*lateralError;
                deltaTurn = discountCoef*deltaTurn + (1.0-discountCoef)*5.0*turnError;
            }

            if (isValid) {
                std::cout << deltaStep << " " << deltaLateral << " " << deltaTurn << std::endl;
                std::cout << motionCapture.pos.azimuth << " ---> " << turnError << std::endl;
            } else {
                std::cout << "No motion capture" << std::endl;
            }

            dynamicParams("step") = 6.0; //Treadmill speed=4
            dynamicParams("turn") = -4.0 + deltaTurn;
            dynamicParams("lateral") = 0.0 + deltaLateral;

            if (dynamicParams("step") > 12.0) dynamicParams("step") = 12.0;
            if (dynamicParams("step") < 2.0) dynamicParams("step") = 2.0;
            if (dynamicParams("lateral") > 8.0) dynamicParams("lateral") = 8.0;
            if (dynamicParams("lateral") < -8.0) dynamicParams("lateral") = -8.0;
            if (dynamicParams("turn") > 50.0) dynamicParams("turn") = 50.0;
            if (dynamicParams("turn") < -50.0) dynamicParams("turn") = -50.0;

            walk.exec(1.0/freq, dynamicParams, staticParams);
            Leph::VectorLabel outputs = walk.lastOutputs();
            forwardMotorsOrders(motors, outputs);
            std::this_thread::sleep_for(
                    std::chrono::milliseconds((int)(1000/freq)));
        }
        std::cout << "Walk disabled" << std::endl;
        dynamicParams("enabled") = 0;
        runWalk(motors, walk, staticParams, dynamicParams, 1.0);

        /*
        for (double t=0.0;t<=20.0;t+=1.0/freq) {
            motionCapture.tick(1.0/freq);
            std::cout << retrieveMotorsAngle(motors) << std::endl;
            std::cout << motionCapture.pos.isValid << std::endl;
            std::cout << motionCapture.pos.x << " " 
                      << motionCapture.pos.y << " "
                      << motionCapture.pos.z << " "
                      << motionCapture.pos.azimuth << std::endl;
            std::this_thread::sleep_for(
                    std::chrono::milliseconds((int)(1000/freq)));
        }
        */
        
        std::cout << "Stopping" << std::endl;
        motors->stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } catch (std::string err) {
        std::cout << "Exception : " << err << std::endl;
    }

    return 0;
}

