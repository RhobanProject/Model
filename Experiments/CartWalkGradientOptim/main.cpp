#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Utils/CircularBuffer.hpp"

//UTILS
#include "MotionCapture.hpp"
#include "SDKConnection.hpp"
#include "SDKInterface.hpp"
  
/**
 * Return current time in milliseconds
 * (Relative to system start)
 */
unsigned long now()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main()
{
    //Init SDK connection
    Leph::SDKConnection sdkConnection;
    
    //Init CartWalk
    Leph::CartWalkProxy walk;
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();

    //Select subset of all static parameters
    Leph::VectorLabel usedPameters({
        "timeGain", 
        "riseGain",
        "swingGain",
        "swingPhase",
        "xOffset",
        "yOffset",
        "zOffset",
        "hipOffset",
        "yLat",
        "swingForce",
        "riseRatio"});
    dynamicParams("step") = 12.0;
    
    //Init Motion Capture
    Rhoban::MotionCapture motionCapture;
    motionCapture.setCaptureStream("tcp://192.168.16.10:3232");
    motionCapture.averageCoefPos = 0.7;
    motionCapture.maxInvalidTick = 20;

    //Servo target reference
    Leph::VectorLabel posReferences(
        "forward", -0.336085,
        "lateral", -0.0763521,
        "turn", 0.0);
    //Servo proportional gain
    Leph::VectorLabel servoGains(
        "forward", 20.0,
        "lateral", 125.0,
        "turn", 1.25);
    //Walk orders delta
    Leph::VectorLabel deltas(
        "forward", 0.0,
        "lateral", 0.0,
        "turn", 0.0);
    //Delta smoothing coefficient
    Leph::VectorLabel discountCoef(
        "coef", 0.8);
    //Motion capture error from target
    Leph::VectorLabel mocapPosError(
        "forward", 0.0,
        "lateral", 0.0,
        "turn", 0.0);
    //Is motion capture valid
    Leph::VectorLabel mocapIsValid(
        "isValid", 0);

    //Init sensors retrieving
    Leph::VectorLabel sensors = sdkConnection.getSensorValues();
    
    //Init CLI interface
    std::string statusEnabled = "Walk is Disabled";
    Leph::SDKInterface interface(sdkConnection, "CartWalk Gradient Optim");
    interface.addParameters("Dynamic walk parameters", dynamicParams);
    interface.addParameters("Static walk parameters", staticParams);
    interface.addParameters("Servo gain", servoGains);
    interface.addParameters("Reference positions", posReferences);
    interface.addMonitors("Motion Capture isValid", mocapIsValid);
    interface.addMonitors("Motion Capture position error", mocapPosError);
    interface.addMonitors("Deltas", deltas);
    interface.addMonitors("Sensors", sensors);
    interface.addStatus(statusEnabled);
    interface.addBinding(' ', "Toggle walk enable", [&dynamicParams, &statusEnabled](){
        dynamicParams("enabled") = !dynamicParams("enabled");
        if (dynamicParams("enabled")) {
            statusEnabled = "Walk is Enabled";
        } else {
            statusEnabled = "Walk is Disabled";
        }
    });

    //Init circular buffer
    Leph::CircularBuffer buffer(50*20);

    //Init VectorLabel for fitness
    Leph::VectorLabel fitness(5);
    interface.addMonitors("Fitness", fitness);
    
    //Main loop
    const double freq = 50.0;
    unsigned int countLoop = 0;
    unsigned long timerOld = now();
    unsigned long timerNew = now();
    while (interface.tick((countLoop % 20) == 0)) {
        //Update sensors
        sensors = sdkConnection.getSensorValues();
        //Update motion capture
        motionCapture.tick(1.0/freq);
        mocapIsValid("isValid") = motionCapture.pos.isValid;
        mocapPosError("forward") = motionCapture.pos.z - posReferences("forward");
        mocapPosError("lateral") = motionCapture.pos.x - posReferences("lateral");
        mocapPosError("turn") = motionCapture.pos.azimuth - posReferences("turn");

        //Compute fitness candidates
        if (mocapIsValid("isValid")) {
            Leph::VectorLabel fitnessData(
                "mocap lateral", motionCapture.pos.x*100,
                "mocap turn", motionCapture.pos.azimuth,
                "sensors x", sensors("GyroX"),
                "sensors y", sensors("GyroY"),
                "sensors z", sensors("GyroZ"));
            buffer.add(fitnessData);
        }

        //Update fitness candidates
        if (buffer.count() > 0 && countLoop % 50 == 0) {
            fitness = buffer.variance();
        }

        //Update delta (proportional servo and smoothing)
        if (mocapIsValid("isValid")) {
            Leph::Vector tmp1 = discountCoef("coef")*deltas.vect();
            Leph::Vector tmp2 = (1.0-discountCoef("coef"))
                *(servoGains.vect().array() * mocapPosError.vect().array());
            deltas.vect() = tmp1 + tmp2;
        }

        //Current dynamic walk parameters
        Leph::VectorLabel tmpDynamicParams = dynamicParams;
        tmpDynamicParams("step") += deltas("forward");
        tmpDynamicParams("lateral") += deltas("lateral");
        tmpDynamicParams("turn") += deltas("turn");

        //Generate walk angle outputs
        walk.exec(1.0/freq, tmpDynamicParams, staticParams);
        //Send orders to motors
        Leph::VectorLabel outputs = walk.lastOutputs();
        sdkConnection.setMotorAngles(outputs);

        //Wait for scheduling
        timerNew = now();
        unsigned long waitDelay = 1000/freq - (timerNew - timerOld);
        std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
        timerOld = now();
        countLoop++;
    }

    return 0;
}

