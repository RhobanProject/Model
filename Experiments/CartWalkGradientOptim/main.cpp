#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <random>
#include <fstream>
#include <sstream>
#include <ctime>

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Utils/CircularBuffer.hpp"

//UTILS
#include "MotionCapture.hpp"
#include "SDKConnection.hpp"
#include "SDKInterface.hpp"

//Global random generator
std::mt19937 randomGenerator;
  
/**
 * Return current time in milliseconds
 * (Relative to system start)
 */
unsigned long now()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * Bound given static parameters between 
 * allowed min and max value
 */
void boundParameters(Leph::VectorLabel& staticParams, 
    const Leph::VectorLabel min,
    const Leph::VectorLabel max)
{
    for (size_t i=0;i<staticParams.size();i++) {
        if (staticParams(i) < min(staticParams.getLabel(i))) {
            staticParams(i) = min(staticParams.getLabel(i));
        }
        if (staticParams(i) > max(staticParams.getLabel(i))) {
            staticParams(i) = max(staticParams.getLabel(i));
        }
    }
}

/**
 * Return random generated delta static parameters
 * with variance given by coef*typical parameter length scale
 */
Leph::VectorLabel randDeltaStaticParams(
    const Leph::VectorLabel staticParamsDelta, double coef = 1.0)
{
    Leph::VectorLabel deltas = staticParamsDelta;
    for (size_t i=0;i<staticParamsDelta.size();i++) {
        std::normal_distribution<double> 
            normalDist(0.0, coef*staticParamsDelta(i));
        deltas(i) = normalDist(randomGenerator);
    }

    return deltas;
}

std::string currentDate()
{
    std::ostringstream oss;
    time_t t = time(0);
    struct tm* now = localtime(&t);
    oss << now->tm_year + 1900 << "-";
    oss << std::setfill('0') << std::setw(2);
    oss << now->tm_mon + 1 << "-";
    oss << std::setfill('0') << std::setw(2);
    oss << now->tm_mday << "-";
    oss << std::setfill('0') << std::setw(2);
    oss << now->tm_hour << ":";
    oss << std::setfill('0') << std::setw(2);
    oss << now->tm_min << ":";
    oss << std::setfill('0') << std::setw(2);
    oss << now->tm_sec;

    return oss.str();
}

int main()
{
    //Init SDK connection
    Leph::SDKConnection sdkConnection;

    //Init CartWalk
    Leph::CartWalkProxy walk;
    Leph::VectorLabel allStaticParams = walk.buildStaticParams();
    Leph::VectorLabel allStaticParamsMin = walk.buildStaticParamsMin();
    Leph::VectorLabel allStaticParamsMax = walk.buildStaticParamsMax();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();
    Leph::VectorLabel allStaticParamsDelta = walk.buildStaticParamsDelta();
    dynamicParams("step") = 12.0;

    //Select subset of all static parameters
    Leph::VectorLabel staticParams({
        "timeGain", 
        "riseGain",
        "swingGain",
        "swingPhase",
        "xOffset",
        "yOffset",
        "zOffset",
        "hipOffset",
        //"yLat",
        //"swingForce",
        //"riseRatio",
    });
    staticParams.mergeInter(allStaticParams);
    Leph::VectorLabel staticParamsDelta = staticParams;
    staticParamsDelta.mergeInter(allStaticParamsDelta);
    
    //Init Motion Capture
    Rhoban::MotionCapture motionCapture;
    motionCapture.setCaptureStream("tcp://192.168.16.10:3232");
    motionCapture.averageCoefPos = 0.7;
    motionCapture.maxInvalidTick = 20;

    //Servo target reference
    Leph::VectorLabel posReferences(
        "forward", -0.3861,
        "lateral", 0.1736,
        "turn", -1.6);
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
    interface.addBinding('r', "Generate random params", [&staticParams, &interface, 
        &staticParamsDelta, &allStaticParamsMin, &allStaticParamsMax] () {
        Leph::VectorLabel deltas = randDeltaStaticParams(staticParamsDelta);
        staticParams += deltas;
        boundParameters(staticParams, allStaticParamsMin, allStaticParamsMax);
        interface.drawParamsWin();
    });

    //Init circular buffer
    Leph::CircularBuffer buffer(50*20);

    //Init VectorLabel for fitness
    Leph::VectorLabel fitness({
        "var mocap lateral", 
        "var mocap turn", 
        "var gyro x", 
        "var gyro y",
        "var gyro z"});
    interface.addMonitors("Fitness", fitness);

    //Init VectorLabel for timestamp
    Leph::VectorLabel timestamp("timestamp", now());
    
    //Open loging file
    std::ofstream logFile("log-" + currentDate() + ".csv");
    
    //Main loop
    const double freq = 50.0;
    unsigned int countLoop = 0;
    unsigned long timerOld = now();
    unsigned long timerNew = now();
    while (interface.tick(false)) {
        //Update interface
        if (countLoop%25 == 0) {
            interface.drawMonitorsWin();
        }
        //Update sensors
        sensors = sdkConnection.getSensorValues();
        //Update motion capture
        motionCapture.tick(1.0/freq);
        mocapIsValid("isValid") = motionCapture.pos.isValid;
        mocapPosError("forward") = motionCapture.pos.z - posReferences("forward");
        mocapPosError("lateral") = motionCapture.pos.x - posReferences("lateral");
        mocapPosError("turn") = motionCapture.pos.azimuth - posReferences("turn");
        //Update timestamp
        timestamp("timestamp") = now();

        //Compute fitness candidates
        if (mocapIsValid("isValid")) {
            Leph::VectorLabel fitnessData(
                "var mocap lateral", motionCapture.pos.x*100,
                "var mocap turn", motionCapture.pos.azimuth,
                "var gyro x", sensors("GyroX"),
                "var gyro y", sensors("GyroY"),
                "var gyro z", sensors("GyroZ"));
            buffer.add(fitnessData);
        }

        //Update fitness candidates
        if (buffer.count() > 0 && countLoop % 50 == 0) {
            fitness = buffer.variance();
        }

        //Update delta (proportional servo and smoothing)
        if (mocapIsValid("isValid")) {
            deltas *= discountCoef("coef");
            Leph::VectorLabel tmp = servoGains;
            tmp *= mocapPosError;
            tmp *= (1.0-discountCoef("coef"));
            deltas += tmp;
        }

        //Current dynamic walk parameters
        Leph::VectorLabel tmpDynamicParams = dynamicParams;
        tmpDynamicParams("step") += deltas("forward");
        tmpDynamicParams("lateral") += deltas("lateral");
        tmpDynamicParams("turn") += deltas("turn");

        //Generate walk angle outputs
        walk.exec(1.0/freq, tmpDynamicParams, 
            staticParams + allStaticParams);
        //Send orders to motors
        Leph::VectorLabel outputs = walk.lastOutputs();
        sdkConnection.setMotorAngles(outputs);

        //Writing log
        (timestamp & 
            tmpDynamicParams & 
            staticParams & 
            allStaticParams & 
            fitness & 
            sensors).writeToCSV(logFile);
        //Wait for scheduling
        timerNew = now();
        unsigned long waitDelay = 1000/freq - (timerNew - timerOld);
        std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(1000/freq)));
        timerOld = now();
        countLoop++;
    }
    interface.quit();

    //Closing loging file
    logFile.close();

    return 0;
}

