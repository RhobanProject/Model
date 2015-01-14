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

int main()
{
    //Init SDK connection
    Leph::SDKConnection sdkConnection;
    
    //Init CartWalk
    Leph::CartWalkProxy walk;
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();
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
    Leph::CircularBuffer buffer1(50*10);
    Leph::CircularBuffer buffer2(50*20);
    Leph::CircularBuffer buffer3(50*30);

    //Init VectorLabel for fitness
    Leph::VectorLabel fitness1(5);
    Leph::VectorLabel fitness2(5);
    Leph::VectorLabel fitness3(5);
    interface.addMonitors("Fitness 1", fitness1);
    interface.addMonitors("Fitness 2", fitness2);
    interface.addMonitors("Fitness 3", fitness3);
    
    //Main loop
    const double freq = 50.0;
    unsigned int countLoop = 0;
    while (interface.tick((countLoop % 10) == 0)) {
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
                "mocap lateral", motionCapture.pos.x,
                "mocap turn", motionCapture.pos.azimuth,
                "sensors x", sensors("GyroX"),
                "sensors y", sensors("GyroY"),
                "sensors z", sensors("GyroZ"));
            buffer1.add(fitnessData);
            buffer2.add(fitnessData);
            buffer3.add(fitnessData);
        }

        //Update fitness candidates
        if (buffer1.count() > 0) {
            fitness1 = buffer1.variance();
            fitness2 = buffer2.variance();
            fitness3 = buffer3.variance();
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
        std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
        countLoop++;
    }

    return 0;
}

