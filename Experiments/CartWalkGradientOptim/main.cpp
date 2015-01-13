#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <signal.h>
#include <unistd.h>

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

//UTILS
#include "MotionCapture.hpp"
#include "SDKConnection.hpp"

void runWalk(Leph::SDKConnection& sdkConnection, Leph::CartWalkProxy& walk, 
    const Leph::VectorLabel& staticParams, const Leph::VectorLabel& dynamicParams, 
    double duration)
{
    const double freq = 50.0;
    for (double t=0.0;t<=duration;t+=1.0/freq) {
        walk.exec(1.0/freq, dynamicParams, staticParams);
        Leph::VectorLabel outputs = walk.lastOutputs();
        sdkConnection.setMotorAngles(outputs);
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

    const double forwardReference = -0.336085;
    const double turnReference = 0.0;
    const double lateralReference = -0.0763521;
    const double freq = 50.0;
    
    try {
        std::cout << "Connection..." << std::endl;
        Leph::SDKConnection sdkConnection;
        
        std::cout << "Walk disabled" << std::endl;
        dynamicParams("enabled") = 0;
        runWalk(sdkConnection, walk, staticParams, dynamicParams, 2.0);
        std::cout << "Walk enabled" << std::endl;
        dynamicParams("enabled") = 1;
        double deltaStep = 0.0;
        double deltaLateral = 0.0;
        double deltaTurn = 0.0;
        double discountCoef = 0.8;
        while (!askedExit) {
            motionCapture.tick(1.0/freq);
            bool isValid = motionCapture.pos.isValid;
            double forwardError = motionCapture.pos.z - forwardReference;
            double turnError = motionCapture.pos.azimuth - turnReference;
            double lateralError = motionCapture.pos.x - lateralReference;

            if (isValid) {
                deltaStep = discountCoef*deltaStep + (1.0-discountCoef)*20.0*forwardError;
                deltaLateral = discountCoef*deltaLateral + (1.0-discountCoef)*125.0*lateralError;
                deltaTurn = discountCoef*deltaTurn + (1.0-discountCoef)*1.25*turnError;
            }

            if (isValid) {
                std::cout << "forward=" << motionCapture.pos.z << " ";
                std::cout << "lateral=" << motionCapture.pos.azimuth << " ";
                std::cout << "turn=" << motionCapture.pos.x << std::endl;
                std::cout << "Deltas: " << deltaStep << " " << deltaLateral << " " << deltaTurn << std::endl;
            } else {
                std::cout << "No motion capture" << std::endl;
            }

            dynamicParams("step") = 12.0 + deltaStep; //Treadmill speed=4
            dynamicParams("turn") = 0.0 + deltaTurn;
            dynamicParams("lateral") = 0.0 + deltaLateral;

            if (dynamicParams("step") > 30.0) dynamicParams("step") = 30.0;
            if (dynamicParams("step") < 0.0) dynamicParams("step") = 0.0;
            if (dynamicParams("lateral") > 20.0) dynamicParams("lateral") = 20.0;
            if (dynamicParams("lateral") < -20.0) dynamicParams("lateral") = -20.0;
            if (dynamicParams("turn") > 50.0) dynamicParams("turn") = 50.0;
            if (dynamicParams("turn") < -50.0) dynamicParams("turn") = -50.0;

            walk.exec(1.0/freq, dynamicParams, staticParams);
            Leph::VectorLabel outputs = walk.lastOutputs();
            sdkConnection.setMotorAngles(outputs);
            std::this_thread::sleep_for(
                    std::chrono::milliseconds((int)(1000/freq)));
        }
        std::cout << "Walk disabled" << std::endl;
        dynamicParams("enabled") = 0;
        runWalk(sdkConnection, walk, staticParams, dynamicParams, 1.0);

        std::cout << "Stopping" << std::endl;
    } catch (std::string err) {
        std::cout << "Exception : " << err << std::endl;
    }

    return 0;
}

