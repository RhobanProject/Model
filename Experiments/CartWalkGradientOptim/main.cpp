#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <random>
#include <fstream>
#include <sstream>
#include <ctime>

//CODE
#include "Utils/time.h"
#include "Types/VectorLabel.hpp"
#include "Types/MatrixLabel.hpp"
#include "Gradient/FiniteDifferenceGradient.hpp"
#include "CartWalk/CartWalkProxy.hpp"

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

int main()
{
    //Init SDK connection
    Leph::SDKConnection sdkConnection;

    //Init CartWalk and parameters default, min, max and deltas
    Leph::CartWalkProxy walk;
    const Leph::VectorLabel allParamsMin = walk.buildParamsMin();
    const Leph::VectorLabel allParamsMax = walk.buildParamsMax();
    const Leph::VectorLabel allParamsDelta = walk.buildParamsDelta();
    
    //Init a subset of static parameters
    Leph::VectorLabel stateParams(
        "static:timeGain", 0.0,
        //"static:riseGain", 0.0,
        "static:swingGain", 0.0,
        "static:swingPhase", 0.0
        //"static:xOffset", 0.0,
        //"static:yOffset", 0.0,
        //"static:zOffset", 0.0,
        //"static:hipOffset", 0.0
        //"static:yLat", 0.0,
        //"static:swingForce", 0.0,
        //"static:riseRatio", 0.0
    );
    //Load default values
    stateParams.mergeInter(walk.buildParams());

    //Parameters
    Leph::VectorLabel params;
    //Get all dynamic parameters
    params.mergeUnion(walk.buildParams(), "dynamic");
    //Init dynamic parameters values
    params("dynamic:step") = 5.0;
    
    //Servo target reference
    params.append(
        "refpos:step", -0.3861,
        "refpos:lateral", 0.1736,
        "refpos:turn", 0.0
    );
    //Servo proportional gain
    params.append(
        "gain:step", 20.0,
        "gain:lateral", 100.0,
        "gain:turn", 1.0
    );
    //Delta smoothing coefficient
    params.append(
        "smooth:coef", 0.95
    );
    //Parameters random exploration coefficient
    params.append(
        "exploration:coef", 2.0
    );
    //Fitness unstable value
    //Fitness stable time length (seconds)
    //Fitness number of stable evaluation sequences
    //Gradient step learning rate
    params.append(
        "fitness:unstableValue", 4.0,
        "fitness:timeLength", 30.0,
        "fitness:countSeq", 10.0,
        "fitness:learningRate", 2.0
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
    //Init VectorLabel for timestamp
    monitors.append(
        "time:timestamp", now()
    );
    //Init main loop delay
    monitors.append(
        "time:delay", 0.0,
        "time:error", 0.0
    );
    //Time length of stable ongoing sequence
    monitors.append(
        "time:length", 0.0
    );
    //Is real robot is stable
    monitors.append("fitness:isStable", 0.0);
    //The number of stable fitness sequences
    monitors.append("fitness:stableSeq", 0.0);
    
    //Current used parameters 
    //(static params and dynamic params control)
    Leph::VectorLabel currentParams = walk.buildParams();

    //All log VectorLabel
    Leph::VectorLabel log = currentParams;
    log.mergeUnion(monitors);
    log.mergeUnion(stateParams);
    log.mergeUnion(walk.buildOutputs());
    
    //Init Motion Capture
    Rhoban::MotionCapture motionCapture;
    motionCapture.setCaptureStream("tcp://192.168.16.10:3232");
    motionCapture.averageCoefPos = 0.7;
    motionCapture.maxInvalidTick = 50;
    
    //Init stable serie Matrix Label container
    Leph::MatrixLabel stableSerie;
    //Init stable sequence fitness container
    Leph::MatrixLabel fitnessesStable;
    //Init nonstable sequence fitness container
    Leph::MatrixLabel fitnessesUnstable;

    //Init CLI interface
    std::string statusEnabled = "Walk is Disabled";
    std::string statusStable = "Real robot is not stable";
    Leph::SDKInterface interface(sdkConnection, "CartWalk Gradient Optim");
    interface.addParameters("Dynamic walk parameters", params, "dynamic");
    interface.addParameters("Servo gain", params, "gain");
    interface.addParameters("Reference positions", params, "refpos");
    interface.addParameters("Learning", params, "exploration");
    interface.addParameters("Fitness", params, "fitness");
    interface.addMonitors("Motion Capture", monitors, "mocap");
    interface.addMonitors("Motion Capture position error", monitors, "error");
    interface.addMonitors("Deltas", monitors, "delta");
    interface.addMonitors("State", stateParams);
    interface.addMonitors("Static Params", currentParams, "static");
    interface.addMonitors("Fitness", monitors, "fitness");
    interface.addMonitors("Time", monitors, "time");
    interface.addStatus(statusEnabled);
    interface.addStatus(statusStable);
    interface.addBinding(' ', "Toggle walk enable", [&params, &statusEnabled](){
        params("dynamic:enabled") = !params("dynamic:enabled");
        if (params("dynamic:enabled")) {
            statusEnabled = "Walk is Enabled";
        } else {
            statusEnabled = "Walk is Disabled";
        }
    });
    interface.addBinding('f', "Toggle robot stable", [&monitors, &statusStable](){
        monitors("fitness:isStable") = !monitors("fitness:isStable");
        if (monitors("fitness:isStable")) {
            statusStable = "Real robot is STABLE";
        } else {
            statusStable = "Real robot is not stable";
        }
    });
    std::function<void()> randomParamsFunc = [&currentParams, &params, &interface, 
        &stateParams, &allParamsDelta, &allParamsMin, &allParamsMax] () {
        Leph::VectorLabel deltas = randDeltaParams(allParamsDelta, 
            params("exploration:coef"));
        currentParams.mergeInter(stateParams, "static");
        currentParams.addOp(
            Leph::VectorLabel::mergeInter(deltas, stateParams), "static");
        boundParameters(currentParams, allParamsMin, allParamsMax);
        interface.drawParamsWin();
    };
    interface.addBinding('r', "Generate random params", randomParamsFunc);
    interface.addBinding('u', "Set sequence to non stable", [&fitnessesUnstable, &params](){
        Leph::VectorLabel tmpFitness = params.extract("static");
        tmpFitness.append("fitness:sum", params("fitness:unstableValue"));
        fitnessesUnstable.append(tmpFitness);
    });

    //Open loging file
    std::ofstream logFileRaw("log-" 
        + Leph::currentDate() + "_raw.csv");
    std::ofstream logFileLearning("log-" 
        + Leph::currentDate() + "_learning.csv");
    stateParams.rename("static", "state").writeToCSV(logFileLearning);
            
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
        if (monitors("fitness:isStable") /*&& monitors("mocap:isValid")*/) { //TODO
            stableSerie.append(Leph::VectorLabel(
                //"fitness:lateral", monitors("error:lateral")*100, TODO
                //"fitness:AccX", monitors("sensor:AccX"),
                "fitness:AccY", monitors("sensor:AccY")
                //"fitness:AccZ", monitors("sensor:AccZ"),
                //"fitness:GyroX", monitors("sensor:GyroX"),
                //"fitness:GyroY", monitors("sensor:GyroY")
                //"fitness:Roll", monitors("sensor:Roll"),
                //"fitness:Pitch", monitors("sensor:Pitch")
            ));
            monitors("time:length") += 1.0/freq;
        } else {
            stableSerie.clear();
            monitors("time:length") = 0.0;
        }

        //Stop stable sequence when enough time is captured
        //Compute fitness
        if (monitors("time:length") >= params("fitness:timeLength")) {
            //Compute fitness (sensors variance)
            Leph::VectorLabel tmpFitness = Leph::VectorLabel::mergeInter(
                currentParams, stateParams);
            tmpFitness.mergeUnion(stableSerie.stdDev());
            fitnessesStable.append(tmpFitness);
            monitors("fitness:stableSeq")++;
            //Reset stable sequence
            monitors("time:length") = 0.0;
            monitors("fitness:isStable") = 0.0;
            statusStable = "Real robot is not stable";
            stableSerie.clear();
            interface.drawMonitorsWin();
            interface.drawStatusWin();
            //Generate new parameters
            randomParamsFunc();
        }

        //Gradient update
        if (fitnessesStable.size() >= params("fitness:countSeq")) {
            //Stable fitness normalization
            Leph::VectorLabel refFitness = fitnessesStable[0];
            /* TODO no normalization using only var AccY
            fitnessesStable.subOp(refFitness, "fitness");
            Leph::VectorLabel stddevFitness = fitnessesStable.stdDev();
            fitnessesStable.divOp(stddevFitness, "fitness");
            */
            //Merge fitness values to summed scalar
            Leph::FiniteDifferenceGradient gradientAlgo;
            for (size_t i=0;i<fitnessesStable.size();i++) {
                //TODO fitnessesStable[i].append("fitness:sum", fitnessesStable[i].mean("fitness"));
                fitnessesStable[i].writeToCSV(logFileLearning);
            }
            //Static parameter normalization
            fitnessesStable.subOp(refFitness, "static");
            fitnessesStable.divOp(allParamsDelta, "static");
            //Compute gradient
            for (size_t i=0;i<fitnessesStable.size();i++) {
                gradientAlgo.addExperiment(
                    fitnessesStable[i].extract("static").vect(), 
                    fitnessesStable[i]("fitness:AccY"));
                    //fitnessesStable[i]("fitness:sum")); TODO
            }
            Leph::VectorLabel gradient = stateParams;
            gradient.vect() = gradientAlgo.gradient();
            //Checking for invalid gradient
            if (gradient.isNan()) {
                throw std::runtime_error("Gradient is Nan");
            }
            //Gradient un-normalization
            gradient.mulOp(allParamsDelta, "static");
            //Gradient step
            gradient.mulOp(params("fitness:learningRate"));
            gradient.rename("static", "gradient").writeToCSV(logFileLearning);
            stateParams.subOp(gradient, "static");
            stateParams.rename("static", "state").writeToCSV(logFileLearning);
            currentParams.mergeInter(stateParams);
            //Clear fitness container
            fitnessesStable.clear();
            fitnessesUnstable.clear();
            monitors("fitness:stableSeq") = 0;
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
        currentParams.mergeInter(params, "dynamic");
        currentParams.addOp(monitors, "delta", "dynamic");
        
        //Generate walk angle outputs
        walk.exec(1.0/freq, currentParams);
        //Send orders to motors
        Leph::VectorLabel outputs = walk.lastOutputs();
        sdkConnection.setMotorAngles(outputs);

        //Writing log
        log.mergeInter(currentParams);
        log.mergeInter(stateParams.rename("static", "state"));
        log.mergeInter(monitors);
        log.mergeInter(outputs);
        log.writeToCSV(logFileRaw);
        
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
    logFileRaw.close();
    logFileLearning.close();

    return 0;
}

