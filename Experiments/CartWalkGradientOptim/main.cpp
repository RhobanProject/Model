#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

//UTILS
#include "MotionCapture.hpp"
#include "SDKConnection.hpp"
#include "SDKInterface.hpp"

int main()
{
    Leph::SDKConnection sdkConnection;
    
    Leph::CartWalkProxy walk;
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();
    dynamicParams("step") = 12.0;
    
    Rhoban::MotionCapture motionCapture;
    motionCapture.setCaptureStream("tcp://192.168.16.10:3232");
    motionCapture.averageCoefPos = 0.7;
    motionCapture.maxInvalidTick = 20;

    Leph::VectorLabel posReferences(
        "forward", -0.336085,
        "lateral", -0.0763521,
        "turn", 0.0);
    Leph::VectorLabel servoGains(
        "forward", 20.0,
        "lateral", 125.0,
        "turn", 1.25);
    Leph::VectorLabel deltas(
        "forward", 0.0,
        "lateral", 0.0,
        "turn", 0.0);
    Leph::VectorLabel discountCoef(
        "coef", 0.8);
    Leph::VectorLabel mocapPosError(
        "forward", 0.0,
        "lateral", 0.0,
        "turn", 0.0);
    Leph::VectorLabel mocapIsValid(
        "isValid", 0);
    
    std::string statusEnabled = "Walk is Disabled";
    Leph::SDKInterface interface(sdkConnection, "CartWalk Gradient Optim");
    interface.addParameters("Dynamic walk parameters", dynamicParams);
    interface.addParameters("Static walk parameters", staticParams);
    interface.addParameters("Servo gain", servoGains);
    interface.addParameters("Reference positions", posReferences);
    interface.addMonitors("Motion Capture isValid", mocapIsValid);
    interface.addMonitors("Motion Capture position error", mocapPosError);
    interface.addMonitors("Deltas", deltas);
    interface.addStatus(statusEnabled);
    interface.addBinding(' ', "Toggle walk enable", [&dynamicParams, &statusEnabled](){
        dynamicParams("enabled") = !dynamicParams("enabled");
        if (dynamicParams("enabled")) {
            statusEnabled = "Walk is Enabled";
        } else {
            statusEnabled = "Walk is Disabled";
        }
    });
    
    const double freq = 50.0;
    while (interface.tick()) {
        motionCapture.tick(1.0/freq);
        mocapIsValid("isValid") = motionCapture.pos.isValid;
        mocapPosError("forward") = motionCapture.pos.z - posReferences("forward");
        mocapPosError("lateral") = motionCapture.pos.x - posReferences("lateral");
        mocapPosError("turn") = motionCapture.pos.azimuth - posReferences("turn");

        if (mocapIsValid("isValid")) {
            Leph::Vector tmp1 = discountCoef("coef")*deltas.vect();
            Leph::Vector tmp2 = (1.0-discountCoef("coef"))
                *(servoGains.vect().array() * mocapPosError.vect().array());
            deltas.vect() = tmp1 + tmp2;
        }

        Leph::VectorLabel tmpDynamicParams = dynamicParams;
        tmpDynamicParams("step") += deltas("forward");
        tmpDynamicParams("lateral") += deltas("lateral");
        tmpDynamicParams("turn") += deltas("turn");

        walk.exec(1.0/freq, tmpDynamicParams, staticParams);
        Leph::VectorLabel outputs = walk.lastOutputs();
        sdkConnection.setMotorAngles(outputs);
        std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
    }

    return 0;
}

