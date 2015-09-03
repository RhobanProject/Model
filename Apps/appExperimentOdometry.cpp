#include <iostream>
#include <vector>
#include <string>
#include "TimeSeries/SeriesUtils.h"
#include "Utils/LWPRUtils.h"
#include "Plot/Plot.hpp"

/**
 * Statistical data
 */
struct Gaussian {
    double mean;
    double var;
    double count;
};

/**
 * Compute the mean of given Gaussian
 */
static Gaussian mergeGaussian(const std::vector<Gaussian>& vect)
{
    Gaussian result;
    result.mean = 0.0;
    result.var = 0.0;
    result.count = 0.0;

    for (size_t i=0;i<vect.size();i++) {
        result.mean += vect[i].mean;
        result.var += vect[i].var;
        result.count += vect[i].count;
    }
    result.mean /= (double)vect.size();
    result.var /= pow((double)vect.size(), 2);
    result.count /= (double)vect.size();

    return result;
}

/**
 * Compute and return confidence bounds
 */
static double confidenceBounds(const Gaussian& g)
{
    return 1.96*sqrt(g.var/g.count);
}

/**
 * Setup 3 models, and do lot of good things
 */
static void setUpModels(const Leph::MatrixLabel& logs, 
    Leph::ModelSeries& modelWithMocap,
    Leph::ModelSeries& modelNoMocap,
    Leph::ModelSeries& modelNoSensor,
    bool doOptimization,
    bool doLearning,
    int maxIteration,
    double timeLearning, 
    double timeTesting,
    bool isQuiet)
{
    //Initialize ModelSeries
    if (!isQuiet) std::cout << "Initiating ModelSeries" << std::endl;
    Leph::initModelSeries(modelWithMocap, true, true, true);
    Leph::initModelSeries(modelNoMocap, false, true, false);
    Leph::initModelSeries(modelNoSensor, false, false, true);
    
    //Load data into TimeSeries
    if (!isQuiet) std::cout << "Injecting data" << std::endl;
    for (size_t i=0;i<logs.size();i++) {
        double time = logs[i]("time:timestamp")/1000.0;
        Leph::appendModelSeries(modelWithMocap, time, logs[i]);
        Leph::appendModelSeries(modelNoMocap, time, logs[i]);
        Leph::appendModelSeries(modelNoSensor, time, logs[i]);
    }

    //Compute all values throught Concepts graph
    if (!isQuiet) std::cout << "Computing concepts" << std::endl;
    modelWithMocap.propagateConcepts();
    modelNoMocap.propagateConcepts();
    modelNoSensor.propagateConcepts();
        
    //Learn Regression models
    if (doOptimization || doLearning) {
        if (!isQuiet) std::cout << "Learning regressions" << std::endl;
        double beginTime = modelWithMocap.series("mocap_x").timeMin();
        double endTime = modelWithMocap.series("mocap_x").timeMax();
        double beginLearnTime = beginTime + 5.0;
        double endLearnTime = timeLearning;
        double beginTestTime = timeTesting;
        double endTestTime = endTime - 5.0;
        if (doOptimization) {
            modelWithMocap.regressionsOptimizeParameters(
                beginLearnTime, endLearnTime, 
                beginTestTime, endTestTime, maxIteration, isQuiet);
        }
        if (doLearning) {
            modelWithMocap.regressionsLearn(beginLearnTime, endLearnTime);
        }
        //Saving Regressions
        modelWithMocap.regressionsSave("/tmp/");
        //Displaying regression prediction error
        if (!isQuiet) {
            std::cout << "Regressions MSE:" << std::endl;
            modelWithMocap.regressionsPrintMSE(beginTestTime, endTestTime);
        }
    }
    
    //Loading regressions
    modelWithMocap.regressionsLoad("/tmp/");
    modelNoMocap.regressionsLoad("/tmp/");
    modelNoSensor.regressionsLoad("/tmp/");

    //Propagate regressions
    modelNoMocap.propagateRegressions();
    modelNoSensor.regression("model_direct_delta_x_left")
        .computePropagate(&modelWithMocap.series("delta_mocap_x_on_support_left"));
    modelNoSensor.regression("model_direct_delta_y_left")
        .computePropagate(&modelWithMocap.series("delta_mocap_x_on_support_left"));
    modelNoSensor.regression("model_direct_delta_theta_left")
        .computePropagate(&modelWithMocap.series("delta_mocap_x_on_support_left"));
    modelNoSensor.regression("model_direct_delta_x_right")
        .computePropagate(&modelWithMocap.series("delta_mocap_x_on_support_right"));
    modelNoSensor.regression("model_direct_delta_y_right")
        .computePropagate(&modelWithMocap.series("delta_mocap_x_on_support_right"));
    modelNoSensor.regression("model_direct_delta_theta_right")
        .computePropagate(&modelWithMocap.series("delta_mocap_x_on_support_right"));
    //Propagate concepts
    modelNoMocap.propagateConcepts();
    modelNoSensor.propagateConcepts();
}

/**
 * Process mocal validity
 */
static void processRawDataMocapValidity(Leph::MatrixLabel& logs)
{
    auto func = [&logs](size_t index, int len)
    {
        int i = index;
        int count = 0;
        while (count < abs(len) && i < (int)logs.size() && i >= 0) {
            logs[i]("mocap:check") = 0.0;
            count++;
            if (len > 0) {
                i++;
            } else {
                i--;
            }
        }
    };

    size_t indexStart = -1;
    size_t indexStop = -1;
    bool isInvalid = false;
    for (size_t i=0;i<logs.size();i++) {
        logs[i].append("mocap:check", 1.0);
    }
    for (size_t i=0;i<logs.size();i++) {
        if (logs[i]("mocap:is_valid") < 0.5) {
            if (isInvalid) {
            } else {
                indexStart = i;
            }
            isInvalid = true;
        } else {
            if (isInvalid) {
                indexStop = i;
                if (indexStop-indexStart > 20) {
                    func(indexStart, 2*(indexStop-indexStart));
                    func(indexStop, 2*(indexStop-indexStart));
                    func(indexStart, 2*(indexStart-indexStop));
                }
            } else {
            }
            isInvalid = false;
        }
    }
}

/**
 * Plot given models
 */
static void plotModels(
    const Leph::ModelSeries& modelWithMocap, 
    const Leph::ModelSeries& modelNoMocap,
    const Leph::ModelSeries& modelNoSensor)
{
    Leph::Plot plot;
    
    plot.add(modelWithMocap.series("mocap_is_valid"));
    plot.add(modelWithMocap.series("mocap_is_data"));
    plot.add(modelWithMocap.series("mocap_x"));
    plot.add(modelWithMocap.series("mocap_y"));
    plot.add(modelWithMocap.series("mocap_theta"));
    plot.plot("time", "all").render();
    plot.clear();
    
    plot.add(modelWithMocap.series("mocap_is_valid"));
    plot.add(modelWithMocap.series("mocap_is_data"));
    plot.add(modelWithMocap.series("is_support_foot_left"));
    plot.add(modelWithMocap.series("mocap_x"));
    plot.add(modelWithMocap.series("mocap_y"));
    plot.add(modelWithMocap.series("mocap_theta"));
    plot.add(modelWithMocap.series("delta_mocap_x_on_support_left"));
    plot.add(modelWithMocap.series("delta_mocap_x_on_support_right"));
    plot.add(modelWithMocap.series("walk_step"));
    plot.add(modelWithMocap.series("walk_lateral"));
    plot.add(modelWithMocap.series("walk_turn"));
    plot.plot("time", "all").render();
    plot.clear();
    
    plot.add(modelWithMocap.series("mocap_is_valid"));
    plot.add(modelWithMocap.series("is_support_foot_left"));
    plot.add(modelWithMocap.series("mocap_x"));
    plot.add(modelWithMocap.series("mocap_y"));
    plot.add(modelWithMocap.series("head_x"));
    plot.add(modelWithMocap.series("head_y"));
    plot.add(modelWithMocap.series("sensor_roll"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("x_mocap_delta_left", modelWithMocap.series("delta_mocap_x_on_support_left"));
    plot.add("x_model_delta_left", modelNoMocap.series("delta_head_x_on_support_left"));
    plot.add("x_learn_delta_left", modelNoMocap.series("delta_mocap_x_on_support_left"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("x_mocap_delta_right", modelWithMocap.series("delta_mocap_x_on_support_right"));
    plot.add("x_model_delta_right", modelNoMocap.series("delta_head_x_on_support_right"));
    plot.add("x_learn_delta_right", modelNoMocap.series("delta_mocap_x_on_support_right"));
    plot.plot("time", "all").render();
    plot.clear();
    
    plot.add("y_mocap_delta_left", modelWithMocap.series("delta_mocap_y_on_support_left"));
    plot.add("y_model_delta_left", modelNoMocap.series("delta_head_y_on_support_left"));
    plot.add("y_learn_delta_left", modelNoMocap.series("delta_mocap_y_on_support_left"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("y_mocap_delta_right", modelWithMocap.series("delta_mocap_y_on_support_right"));
    plot.add("y_model_delta_right", modelNoMocap.series("delta_head_y_on_support_right"));
    plot.add("y_learn_delta_right", modelNoMocap.series("delta_mocap_y_on_support_right"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("theta_mocap_delta_left", modelWithMocap.series("delta_mocap_theta_on_support_left"));
    plot.add("theta_model_delta_left", modelNoMocap.series("delta_head_theta_on_support_left"));
    plot.add("theta_learn_delta_left", modelNoMocap.series("delta_mocap_theta_on_support_left"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("theta_mocap_delta_right", modelWithMocap.series("delta_mocap_theta_on_support_right"));
    plot.add("theta_model_delta_right", modelNoMocap.series("delta_head_theta_on_support_right"));
    plot.add("theta_learn_delta_right", modelNoMocap.series("delta_mocap_theta_on_support_right"));
    plot.plot("time", "all").render();
    plot.clear();
        
    plot.add("mocap_x", modelWithMocap.series("integrated_mocap_x"));
    plot.add("model_x", modelNoMocap.series("integrated_head_x"));
    plot.add("learn_x", modelNoMocap.series("integrated_mocap_x"));
    plot.plot("time", "all").render();
    plot.clear();

    plot.add("mocap_y", modelWithMocap.series("integrated_mocap_y"));
    plot.add("model_y", modelNoMocap.series("integrated_head_y"));
    plot.add("learn_y", modelNoMocap.series("integrated_mocap_y"));
    plot.plot("time", "all").render();

    plot.clear();
    plot.add("mocap_theta", modelWithMocap.series("integrated_mocap_theta"));
    plot.add("model_theta", modelNoMocap.series("integrated_head_theta"));
    plot.add("learn_theta", modelNoMocap.series("integrated_mocap_theta"));
    plot.plot("time", "all").render();
    plot.clear();

    Leph::plotPhase(plot, modelWithMocap, "integrated_mocap_x", {"integrated_mocap_y"});
    Leph::plotPhase(plot, modelNoMocap, "integrated_mocap_x", {"integrated_mocap_y"});
    Leph::plotPhase(plot, modelNoMocap, "integrated_head_x", {"integrated_head_y"});
    plot
        .plot("integrated_mocap_x", "integrated_mocap_y", Leph::Plot::LinesPoints, "time")
        .plot("integrated_head_x", "integrated_head_y")
        .render();
    plot.clear();
}

int main()
{
    //Learn and Test logs filename container
    std::vector<std::string> fileLogsLearn = {
        "../../These/Data/model_2015-09-02-17-52-53.log",
        //"../../These/Data/model_2015-09-02-18-02-44.log",
        //"../../These/Data/model_2015-09-02-18-13-59.log",
        //"../../These/Data/model_2015-09-02-16-58-24.log",
    };
    std::vector<std::string> fileLogsTest = {
        "../../These/Data/model_2015-09-02-18-26-24.log",
        "../../These/Data/model_2015-09-02-18-27-54.log",
        "../../These/Data/model_2015-09-02-18-56-25.log",
        "../../These/Data/model_2015-09-02-19-01-34.log",
    };
    std::vector<Leph::MatrixLabel> dataLogsLearn;
    std::vector<Leph::MatrixLabel> dataLogsTest;

    //Load data from model logs
    double dataTimeLearnMin = -1.0;
    double dataTimeLearnMax = -1.0;
    double dataTimeTestMin = -1.0;
    double dataTimeTestMax = -1.0;
    std::cout << "Loading data" << std::endl;
    for (size_t i=0;i<fileLogsLearn.size();i++) {
        dataLogsLearn.push_back(Leph::MatrixLabel());
        dataLogsLearn.back().load(fileLogsLearn[i]);
        processRawDataMocapValidity(dataLogsLearn.back());
        //Print data informations
        size_t size = dataLogsLearn.back().size();
        size_t dim = dataLogsLearn.back().dimension();
        double tMin = dataLogsLearn.back()[0]("time:timestamp")/1000.0;
        double tMax = dataLogsLearn.back()[size-1]("time:timestamp")/1000.0;
        std::cout << "Loaded Learn " << fileLogsLearn[i] << ": "
            << size << " points with " 
            << dim << " entries from t="
            << tMin << "..." << tMax << std::endl;
        if (dataTimeLearnMin < 0.0 || dataTimeLearnMin > tMin) {
            dataTimeLearnMin = tMin;
        }
        if (dataTimeLearnMax < 0.0 || dataTimeLearnMax < tMax) {
            dataTimeLearnMax = tMax;
        }
    }
    for (size_t i=0;i<fileLogsTest.size();i++) {
        dataLogsTest.push_back(Leph::MatrixLabel());
        dataLogsTest.back().load(fileLogsTest[i]);
        processRawDataMocapValidity(dataLogsTest.back());
        //Print data informations
        size_t size = dataLogsTest.back().size();
        size_t dim = dataLogsTest.back().dimension();
        double tMin = dataLogsTest.back()[0]("time:timestamp")/1000.0;
        double tMax = dataLogsTest.back()[size-1]("time:timestamp")/1000.0;
        std::cout << "Loaded Test " << fileLogsTest[i] << ": "
            << size << " points with " 
            << dim << " entries from t="
            << tMin << "..." << tMax << std::endl;
        if (dataTimeTestMin < 0.0 || dataTimeTestMin > tMin) {
            dataTimeTestMin = tMin;
        }
        if (dataTimeTestMax < 0.0 || dataTimeTestMax < tMax) {
            dataTimeTestMax = tMax;
        }
    }
    std::cout << "dataTimeLearnMin=" << dataTimeLearnMin << " dataTimeLearnMax=" << dataTimeLearnMax << std::endl;
    std::cout << "dataTimeTestMin=" << dataTimeTestMin << " dataTimeTestMax=" << dataTimeTestMax << std::endl;
    double dataTimeLearnLength = dataTimeLearnMax - dataTimeLearnMin;
    double dataTimeTestLength = dataTimeTestMax - dataTimeTestMin;
    double dataTimeLearnMiddle = 0.5*dataTimeLearnMax + 0.5*dataTimeLearnMin;
    double dataTimeTestMiddle = 0.5*dataTimeTestMax + 0.5*dataTimeTestMin;

    //Shift mocap data offset to zero
    for (size_t i=1;i<dataLogsLearn[0].size();i++) {
        dataLogsLearn[0][i]("mocap:x") -= dataLogsLearn[0][0]("mocap:x");
        dataLogsLearn[0][i]("mocap:y") -= dataLogsLearn[0][0]("mocap:y");
        dataLogsLearn[0][i]("mocap:z") -= dataLogsLearn[0][0]("mocap:z");
        dataLogsLearn[0][i]("mocap:azimuth") -= dataLogsLearn[0][0]("mocap:azimuth");
        dataLogsLearn[0][i]("sensor:gyro_yaw") -= dataLogsLearn[0][0]("sensor:gyro_yaw");
    }

    Leph::Plot plotData;
    /*
    for (double time=dataTimeLearnMin+8.0;time<=dataTimeLearnMiddle;time+=dataTimeLearnLength/20.0) {
        //Init statitics container
        std::vector<Gaussian> modelX;
        std::vector<Gaussian> modelY;
        std::vector<Gaussian> modelTheta;
        std::vector<Gaussian> modelLearnX;
        std::vector<Gaussian> modelLearnY;
        std::vector<Gaussian> modelLearnTheta;
        std::vector<Gaussian> walkX;
        std::vector<Gaussian> walkY;
        std::vector<Gaussian> walkTheta;
        std::vector<Gaussian> walkLearnX;
        std::vector<Gaussian> walkLearnY;
        std::vector<Gaussian> walkLearnTheta;
        for (size_t i=0;i<dataLogsLearn.size();i++) {
            for (int k=1;k<=1;k++) {
                //Init models
                Leph::ModelSeries modelWithMocap;
                Leph::ModelSeries modelNoMocap;
                Leph::ModelSeries modelNoSensor;
                setUpModels(dataLogsLearn[i], 
                    modelWithMocap,
                    modelNoMocap,
                    modelNoSensor,
                    true, //doOptimization
                    false, //doLearning
                    100, //maxIteration
                    time, //timeLearning
                    dataTimeLearnMiddle, //timeTesting
                    true); //isQuiet

                //Lambda computing error statistics on given
                //TimeSeries name
                auto func = [&modelWithMocap](
                    Leph::ModelSeries& model, 
                    const std::string& name1, 
                    const std::string& name2, 
                    bool doSufixe) -> Gaussian 
                {
                    //Time interval
                    double beginTime = modelWithMocap.series("mocap_x").timeMin() + 10.0;
                    double endTime = modelWithMocap.series("mocap_x").timeMax() - 10.0;
                    //Compute error
                    double meanErrorLeft;
                    double varLeft;
                    int countLeft;
                    double meanErrorRight;
                    double varRight;
                    int countRight;
                    Leph::seriesCompare(
                        modelWithMocap.series(name1 + "_left"), 
                        model.series(name2 + (doSufixe ? "_left" : "")), 
                        beginTime, endTime, 
                        meanErrorLeft, varLeft, countLeft);
                    Leph::seriesCompare(
                        modelWithMocap.series(name1 + "_right"), 
                        model.series(name2 + (doSufixe ? "_right" : "")), 
                        beginTime, endTime, 
                        meanErrorRight, varRight, countRight);
                    //Return stats
                    Gaussian stats;
                    stats.mean = 0.5*meanErrorLeft + 0.5*meanErrorRight;
                    stats.var = 0.5*0.5*varLeft + 0.5*0.5*varRight;
                    stats.count = 0.5*countLeft + 0.5*countRight;
                    return stats;
                };

                //Computing statistics
                std::cout << "Generated time=" << time << " log=" << i << " k=" << k << std::endl;
                modelX.push_back(func(modelNoMocap, "delta_mocap_x_on_support", "delta_head_x_on_support", true));
                modelY.push_back(func(modelNoMocap, "delta_mocap_y_on_support", "delta_head_y_on_support", true));
                modelTheta.push_back(func(modelNoMocap, "delta_mocap_theta_on_support", "delta_head_theta_on_support", true));
                modelLearnX.push_back(func(modelNoMocap, "delta_mocap_x_on_support", "delta_mocap_x_on_support", true));
                modelLearnY.push_back(func(modelNoMocap, "delta_mocap_y_on_support", "delta_mocap_y_on_support", true));
                modelLearnTheta.push_back(func(modelNoMocap, "delta_mocap_theta_on_support", "delta_mocap_theta_on_support", true));
                walkX.push_back(func(modelNoSensor, "delta_mocap_x_on_support", "walk_step", false));
                walkY.push_back(func(modelNoSensor, "delta_mocap_y_on_support", "walk_lateral", false));
                walkTheta.push_back(func(modelNoSensor, "delta_mocap_theta_on_support", "walk_turn", false));
                walkLearnX.push_back(func(modelNoSensor, "delta_mocap_x_on_support", "delta_mocap_x_on_support", true));
                walkLearnY.push_back(func(modelNoSensor, "delta_mocap_y_on_support", "delta_mocap_y_on_support", true));
                walkLearnTheta.push_back(func(modelNoSensor, "delta_mocap_theta_on_support", "delta_mocap_theta_on_support", true));
            }
        }
        Gaussian modelXGaussian = mergeGaussian(modelX);
        Gaussian modelYGaussian = mergeGaussian(modelY);
        Gaussian modelThetaGaussian = mergeGaussian(modelTheta);
        Gaussian modelLearnXGaussian = mergeGaussian(modelLearnX);
        Gaussian modelLearnYGaussian = mergeGaussian(modelLearnY);
        Gaussian modelLearnThetaGaussian = mergeGaussian(modelLearnTheta);
        Gaussian walkXGaussian = mergeGaussian(walkX);
        Gaussian walkYGaussian = mergeGaussian(walkY);
        Gaussian walkThetaGaussian = mergeGaussian(walkTheta);
        Gaussian walkLearnXGaussian = mergeGaussian(walkLearnX);
        Gaussian walkLearnYGaussian = mergeGaussian(walkLearnY);
        Gaussian walkLearnThetaGaussian = mergeGaussian(walkLearnTheta);
        plotData.add(Leph::VectorLabel(
            "time", time,
            "model_x_mean", modelXGaussian.mean,
            "model_x_bound", confidenceBounds(modelXGaussian),
            "model_y_mean", modelYGaussian.mean,
            "model_y_bound", confidenceBounds(modelYGaussian),
            "model_theta_mean", modelThetaGaussian.mean,
            "model_theta_bound", confidenceBounds(modelThetaGaussian),
            "model_learn_x_mean", modelLearnXGaussian.mean,
            "model_learn_x_bound", confidenceBounds(modelLearnXGaussian),
            "model_learn_y_mean", modelLearnYGaussian.mean,
            "model_learn_y_bound", confidenceBounds(modelLearnYGaussian),
            "model_learn_theta_mean", modelLearnThetaGaussian.mean,
            "model_learn_theta_bound", confidenceBounds(modelLearnThetaGaussian),
            "walk_x_mean", walkXGaussian.mean,
            "walk_x_bound", confidenceBounds(walkXGaussian),
            "walk_y_mean", walkYGaussian.mean,
            "walk_y_bound", confidenceBounds(walkYGaussian),
            "walk_theta_mean", walkThetaGaussian.mean,
            "walk_theta_bound", confidenceBounds(walkThetaGaussian),
            "walk_learn_x_mean", walkLearnXGaussian.mean,
            "walk_learn_x_bound", confidenceBounds(walkLearnXGaussian),
            "walk_learn_y_mean", walkLearnYGaussian.mean,
            "walk_learn_y_bound", confidenceBounds(walkLearnYGaussian),
            "walk_learn_theta_mean", walkLearnThetaGaussian.mean,
            "walk_learn_theta_bound", confidenceBounds(walkLearnThetaGaussian)
        ));
    }
    plotData.plot("time", "all").render();
    plotData.clear();
    */
    
    //Optimize best model
    {
    Leph::ModelSeries tmpModelWithMocap;
    Leph::ModelSeries tmpModelNoMocap;
    Leph::ModelSeries tmpModelNoSensor;
    setUpModels(dataLogsLearn.front(), 
        tmpModelWithMocap,
        tmpModelNoMocap,
        tmpModelNoSensor,
        true, //doOptimization
        false, //doLearning
        100, //maxIteration
        dataTimeLearnMiddle, //timeLearning
        dataTimeLearnMiddle, //timeTesting
        false); //isQuiet
    plotModels(tmpModelWithMocap, tmpModelNoMocap, tmpModelNoSensor);
    }
    {
    Leph::ModelSeries tmpModelWithMocap;
    Leph::ModelSeries tmpModelNoMocap;
    Leph::ModelSeries tmpModelNoSensor;
    setUpModels(dataLogsTest.front(), 
        tmpModelWithMocap,
        tmpModelNoMocap,
        tmpModelNoSensor,
        false, //doOptimization
        false, //doLearning
        100, //maxIteration
        dataTimeLearnMiddle, //timeLearning
        dataTimeLearnMiddle, //timeTesting
        false); //isQuiet
    plotModels(tmpModelWithMocap, tmpModelNoMocap, tmpModelNoSensor);
    }

    for (size_t i=0;i<dataLogsTest.size();i++) {
        //Init models
        Leph::ModelSeries modelWithMocap;
        Leph::ModelSeries modelNoMocap;
        Leph::ModelSeries modelNoSensor;
        setUpModels(dataLogsTest[i], 
            modelWithMocap,
            modelNoMocap,
            modelNoSensor,
            false, //doOptimization
            false, //doLearning
            500, //maxIteration
            dataTimeLearnMiddle, //timeLearning
            dataTimeLearnMiddle, //timeTesting
            true); //isQuiet

        //Lambda computing cartesian distance between real mocap position 
        //and integreted one at given time.
        auto funcPos = [&modelWithMocap](
            Leph::ModelSeries& model, 
            const std::string& name, 
            double time) -> double 
        {
            if (
                !modelWithMocap.series("integrated_mocap_x").isTimeValid(time) ||
                !modelWithMocap.series("integrated_mocap_y").isTimeValid(time) ||
                !model.series(name + "_x").isTimeValid(time) ||
                !model.series(name + "_y").isTimeValid(time)
            ) {
                return -1.0;
            }
            double mocapX = modelWithMocap.series("integrated_mocap_x").get(time);
            double mocapY = modelWithMocap.series("integrated_mocap_y").get(time);
            double posX = model.series(name + "_x").get(time);
            double posY = model.series(name + "_y").get(time);

            return sqrt(pow(mocapX-posX, 2) + pow(mocapY-posY, 2));
        }; 

        size_t indexUp = modelWithMocap.series("integrated_mocap_x").size();
        size_t indexLow = 0;
        for (size_t j=indexLow;j<indexUp;j++) {
            double t = modelWithMocap.series("integrated_mocap_x").at(j).time;
            plotData.add(Leph::VectorLabel(
                "time", t,
                "model", funcPos(modelNoMocap, "integrated_head", t),
                "model_learn", funcPos(modelNoMocap, "integrated_mocap", t),
                "walk_learn", funcPos(modelNoSensor, "integrated_mocap", t)
            ));
        }
    }
    plotData.plot("time", "all").render();
    plotData.clear();

    return 0;
}

