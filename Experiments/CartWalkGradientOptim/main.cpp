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
 * Bound given parameters between 
 * allowed min and max value
 */
void boundParameters(Leph::VectorLabel& params, 
    const Leph::VectorLabel& min,
    const Leph::VectorLabel& max)
{
    for (size_t i=0;i<min.size();i++) {
        const std::string& label = min.getLabel(i);
        if (params.exist(label) && params(label) < min(i)) {
            params(label) = min(i);
        }
        if (params.exist(label) && params(label) > max(i)) {
            params(label) = max(i);
        }
    }
}

/**
 * Return random generated delta parameters
 * with variance given by coef*typical parameter length scale
 */
Leph::VectorLabel randDeltaParams(
    const Leph::VectorLabel paramsDelta, double coef = 1.0)
{
    Leph::VectorLabel deltas = paramsDelta;
    for (size_t i=0;i<paramsDelta.size();i++) {
        std::normal_distribution<double> 
            normalDist(0.0, coef*paramsDelta(i));
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

    //Init CartWalk and parameters default, min, max and deltas
    Leph::CartWalkProxy walk;
    Leph::VectorLabel allParams = walk.buildParams();
    Leph::VectorLabel allParamsMin = walk.buildParamsMin();
    Leph::VectorLabel allParamsMax = walk.buildParamsMax();
    Leph::VectorLabel allParamsDelta = walk.buildParamsDelta();

    //Parameters
    Leph::VectorLabel params;
    //Select subset of all static parameters
    params.append(
        "static:timeGain", 0.0,
        "static:riseGain", 0.0,
        "static:swingGain", 0.0,
        "static:swingPhase", 0.0,
        "static:xOffset", 0.0,
        "static:yOffset", 0.0,
        "static:zOffset", 0.0,
        "static:hipOffset", 0.0
        //"static:yLat", 0.0,
        //"static:swingForce", 0.0,
        //"static:riseRatio", 0.0
    );
    //Get all dynamic parameters
    params.mergeUnion(allParams, "dynamic");
    //Get all selecled static parameters values
    params.mergeInter(allParams);
    //Init dynamic parameters values
    params("dynamic:step") = 5.0;
    
    //Servo target reference
    params.append(
        "refpos:step", -0.3861,
        "refpos:lateral", 0.1736,
        "refpos:turn", -1.6
    );
    //Servo proportional gain
    params.append(
        "gain:step", 20.0,
        "gain:lateral", 125.0,
        "gain:turn", 1.25
    );
    //Delta smoothing coefficient
    params.append(
        "smooth:coef", 0.8
    );
    //Parameters random exploration coefficient
    params.append(
        "exploration:coef", 2.0
    );

    //Monitors
    Leph::VectorLabel monitors;
    //Walk orders delta
    monitors.append(
        "delta:step", 0.0,
        "delta:lateral", 0.0,
        "delta:turn", 0.0
    );
    //Motion capture error from target
    monitors.append(
        "error:step", 0.0,
        "error:lateral", 0.0,
        "error:turn", 0.0
    );
    //Is motion capture valid
    monitors.append(
        "mocap:isValid", 0
    );
    //Init sensors retrieving
    monitors.mergeUnion(sdkConnection.getSensorValues());
    //Init motors retrieving
    monitors.mergeUnion(sdkConnection.getMotorAngles());
    //Init VectorLabel for fitness
    monitors.append(
        "fitness:mocap lateral", 0.0,
        "fitness:mocap turn", 0.0,
        "fitness:gyro x", 0.0,
        "fitness:gyro y", 0.0,
        "fitness:gyro z", 0.0
    );
    //Init VectorLabel for timestamp
    monitors.append(
        "time:timestamp", now()
    );
    //Init main loop delay
    monitors.append(
        "time:delay", 0.0,
        "time:error", 0.0
    );
    
    //Current used parameters
    Leph::VectorLabel currentParams = allParams;
    currentParams.mergeUnion(params);

    //All log VectorLabel
    Leph::VectorLabel log = currentParams;
    log.mergeUnion(monitors);
    log.mergeUnion(walk.buildOutputs());
    
    //Init Motion Capture
    Rhoban::MotionCapture motionCapture;
    motionCapture.setCaptureStream("tcp://192.168.16.10:3232");
    motionCapture.averageCoefPos = 0.7;
    motionCapture.maxInvalidTick = 20;

    //Init CLI interface
    std::string statusEnabled = "Walk is Disabled";
    Leph::SDKInterface interface(sdkConnection, "CartWalk Gradient Optim");
    interface.addParameters("Dynamic walk parameters", params, "dynamic");
    interface.addParameters("Static walk parameters", params, "static");
    interface.addParameters("Servo gain", params, "gain");
    interface.addParameters("Reference positions", params, "refpos");
    interface.addParameters("Learning", params, "exploration");
    interface.addMonitors("Motion Capture", monitors, "mocap");
    interface.addMonitors("Motion Capture position error", monitors, "error");
    interface.addMonitors("Deltas", monitors, "delta");
    interface.addMonitors("Sensors", monitors, "sensor");
    interface.addMonitors("Fitness", monitors, "fitness");
    interface.addMonitors("Time", monitors, "time");
    interface.addStatus(statusEnabled);
    interface.addBinding(' ', "Toggle walk enable", [&params, &statusEnabled](){
        params("dynamic:enabled") = !params("dynamic:enabled");
        if (params("dynamic:enabled")) {
            statusEnabled = "Walk is Enabled";
        } else {
            statusEnabled = "Walk is Disabled";
        }
    });
    interface.addBinding('r', "Generate random params", [&params, &interface, 
        &allParamsDelta, &allParamsMin, &allParamsMax] () {
        Leph::VectorLabel deltas = randDeltaParams(allParamsDelta, 
            params("exploration:coef"));
        params.addOp(deltas, "static");
        boundParameters(params, allParamsMin, allParamsMax);
        interface.drawParamsWin();
    });

    //Init circular buffer
    Leph::CircularBuffer buffer(50*20);

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
        //Update sensors and motors
        monitors.mergeInter(sdkConnection.getSensorValues());
        monitors.mergeInter(sdkConnection.getMotorAngles());
        
        //Update motion capture
        motionCapture.tick(1.0/freq);
        monitors("mocap:isValid") = motionCapture.pos.isValid;
        monitors("error:step") = motionCapture.pos.z - params("refpos:step");
        monitors("error:lateral") = motionCapture.pos.x - params("refpos:lateral");
        monitors("error:turn") = motionCapture.pos.azimuth - params("refpos:turn");
        //Update timestamp
        monitors("time:timestamp") = now();

        //Compute fitness candidates
        if (monitors("mocap:isValid")) {
            Leph::VectorLabel fitnessData(
                "fitness:mocap lateral", motionCapture.pos.x*100,
                "fitness:mocap turn", motionCapture.pos.azimuth,
                "fitness:gyro x", monitors("sensor:GyroX"),
                "fitness:gyro y", monitors("sensor:GyroY"),
                "fitness:gyro z", monitors("sensor:GyroZ"));
            buffer.add(fitnessData);
        }
        
        //Update fitness candidates
        if (buffer.count() > 0 && countLoop % 50 == 0) {
            monitors.mergeUnion(buffer.variance());
        }
        
        //Update delta (proportional servo and smoothing)
        if (monitors("mocap:isValid")) {
            monitors.mulOp(params("smooth:coef"), "delta");
            Leph::VectorLabel tmp;
            tmp.mergeUnion(monitors, "error");
            tmp.mulOp(params, "gain", "error");
            tmp.mulOp((1.0-params("smooth:coef")));
            monitors.addOp(tmp, "error", "delta");
        }
        
        //Current dynamic walk parameters
        currentParams.mergeInter(params);
        currentParams.addOp(monitors, "delta", "dynamic");
        
        //Generate walk angle outputs
        walk.exec(1.0/freq, currentParams);
        //Send orders to motors
        Leph::VectorLabel outputs = walk.lastOutputs();
        sdkConnection.setMotorAngles(outputs);

        //Writing log
        log.mergeInter(currentParams);
        log.mergeInter(monitors);
        log.mergeInter(outputs);
        log.writeToCSV(logFile);
        
        //Wait for scheduling
        timerNew = now();
        unsigned long waitDelay = 0;
        unsigned long timerDiff = timerNew - timerOld;
        if (timerDiff <= 1000/freq) {
            waitDelay = 1000/freq - timerDiff;
        } else {
            monitors("time:error") += 1.0;
        }
        monitors("time:delay") *= 0.8;
        monitors("time:delay") += 0.2*timerDiff;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(waitDelay));
        timerOld = now();
        countLoop++;
    }
    interface.quit();

    //Closing loging file
    logFile.close();

    return 0;
}

