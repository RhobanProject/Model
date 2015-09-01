#include <iostream>
#include "TimeSeries/SeriesUtils.h"

int main()
{
    //Load data from model logs
    std::cout << "Openning log file " << std::endl;
    Leph::MatrixLabel logs;
    std::cout << "Loading data" << std::endl;
    //logs.load("../../These/Data/model_2015-08-26-13-12-23.log");
    //logs.load("../../These/Data/model_2015-08-28-21-41-39.log");
    logs.load("../../These/Data/model_2015-08-30-19-19-54.log");
    //logs.load("../../These/Data/model_2015-08-30-20-50-48.log");
    //logs.load("/tmp/model_2015-08-30-19-19-54.log");
    //logs.load("../../These/Data/model_2015-08-28-21-53-59.log");
    //Print data informations
    std::cout << "Loaded " 
        << logs.size() << " points with " 
        << logs.dimension() << " entries" << std::endl;

    for (double ratio=1.0;ratio<=1.0;ratio+=0.1) {
        //Initialize ModelSeries
        std::cout << "Initiating ModelSeries" << std::endl;
        Leph::ModelSeries modelWithMocap;
        Leph::ModelSeries modelNoMocap;
        Leph::ModelSeries modelNoSensor;
        initModelSeries(modelWithMocap, true, true, true);
        initModelSeries(modelNoMocap, false, true, false);
        initModelSeries(modelNoSensor, false, false, true);

        //Load data into TimeSeries
        std::cout << "Injecting data" << std::endl;
        for (size_t i=0;i<logs.size();i++) {
            double time = logs[i]("time:timestamp")/1000.0;
            appendModelSeries(modelWithMocap, time, logs[i]);
            appendModelSeries(modelNoMocap, time, logs[i]);
            appendModelSeries(modelNoSensor, time, logs[i]);
        }
        
        //Compute all values throught Concepts graph
        std::cout << "Computing concepts" << std::endl;
        modelWithMocap.propagateConcepts();
        modelNoMocap.propagateConcepts();
        modelNoSensor.propagateConcepts();
        
        //Learn Regression models
        std::cout << "Learning regressions" << std::endl;
        double beginTime = modelWithMocap.series("mocap_x").timeMin();
        double endTime = modelWithMocap.series("mocap_x").timeMax();
        double beginLearnTime = beginTime;
        double endLearnTime = ratio*endTime/2.0;
        double beginTestTime = endTime/2.0;
        double endTestTime = endTime;
        modelWithMocap.regressionsOptimizeParameters(
            beginLearnTime, endLearnTime, 
            beginTestTime, endTestTime, 200, true);
        //Saving Regressions
        modelWithMocap.regressionsSave("/tmp/");
        //Displaying regression prediction error
        std::cout << "Regressions MSE:" << std::endl;
        modelWithMocap.regressionsPrintMSE(beginTestTime, endTestTime);

        //Loading regressions
        modelWithMocap.regressionsLoad("/tmp/");
        modelNoMocap.regressionsLoad("/tmp/");
        modelNoSensor.regressionsLoad("/tmp/");

        modelNoMocap.propagateRegressions();
        //modelNoSensor.propagateRegressions();
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
        modelNoMocap.propagateConcepts();
        modelNoSensor.propagateConcepts();

        std::cout << ratio << " " << endLearnTime;
        std::cout << " " << seriesDistance(
            modelWithMocap.series("delta_mocap_x_on_support_left"), 
            modelNoMocap.series("delta_mocap_x_on_support_left"));
        std::cout << " " << seriesDistance(
            modelWithMocap.series("delta_mocap_x_on_support_left"), 
            modelNoMocap.series("delta_head_x_on_support_left"));
        std::cout << " " << seriesDistance(
            modelWithMocap.series("delta_mocap_y_on_support_left"), 
            modelNoMocap.series("delta_mocap_y_on_support_left"));
        std::cout << " " << seriesDistance(
            modelWithMocap.series("delta_mocap_y_on_support_left"), 
            modelNoMocap.series("delta_head_y_on_support_left"));
        std::cout << " " << seriesDistance(
            modelWithMocap.series("delta_mocap_theta_on_support_left"), 
            modelNoMocap.series("delta_mocap_theta_on_support_left"));
        std::cout << " " << seriesDistance(
            modelWithMocap.series("delta_mocap_theta_on_support_left"), 
            modelNoMocap.series("delta_head_theta_on_support_left"));
        std::cout << std::endl;
        
        //Ploting
        Leph::Plot plot;
        plot.add(modelWithMocap.series("mocap_is_valid"));
        plot.add(modelWithMocap.series("mocap_x"));
        plot.add(modelWithMocap.series("mocap_y"));
        plot.add(modelWithMocap.series("mocap_theta"));
        plot.add(modelWithMocap.series("walk_step"));
        plot.add(modelWithMocap.series("walk_lateral"));
        plot.add(modelWithMocap.series("walk_turn"));
        plot.plot("time", "all").render();
        plot.clear();

        plot.add("with_support_foot", modelWithMocap.series("is_support_foot_left"));
        plot.add("with_delta_mocap_x_on_support_left", modelWithMocap.series("delta_mocap_x_on_support_left"));
        plot.add("with_delta_mocap_y_on_support_left", modelWithMocap.series("delta_mocap_y_on_support_left"));
        plot.add("with_delta_mocap_theta_on_support_left", modelWithMocap.series("delta_mocap_theta_on_support_left"));
        plot.add("with_mocap_theta", modelWithMocap.series("mocap_theta"));
        plot.plot("time", "all").render();
        plot.clear();
        
        plot.add("with_delta_mocap_x_on_support_left", modelWithMocap.series("delta_mocap_x_on_support_left"));
        plot.add("with_delta_mocap_y_on_support_left", modelWithMocap.series("delta_mocap_y_on_support_left"));
        //plot.add("with_delta_mocap_theta_on_support_left", modelWithMocap.series("delta_mocap_theta_on_support_left"));
        plot.add(modelWithMocap.series("walk_step"));
        plot.add(modelWithMocap.series("walk_lateral"));
        //plot.add(modelWithMocap.series("walk_turn"));
        plot.plot("time", "all").render();
        plot.clear();
        
        plot.add("with_delta_mocap_x_on_support_left", modelWithMocap.series("delta_mocap_x_on_support_left"));
        plot.add("no_delta_head_x_on_support_left", modelNoMocap.series("delta_head_x_on_support_left"));
        plot.add("no_delta_mocap_x_on_support_left", modelNoMocap.series("delta_mocap_x_on_support_left"));
        plot.add("walk_delta_mocap_x_on_support_left", modelNoSensor.series("delta_mocap_x_on_support_left"));
        plot.plot("time", "all").render();
        plot.clear();
        plot.add("with_delta_mocap_x_on_support_right", modelWithMocap.series("delta_mocap_x_on_support_right"));
        plot.add("no_delta_head_x_on_support_right", modelNoMocap.series("delta_head_x_on_support_right"));
        plot.add("no_delta_mocap_x_on_support_right", modelNoMocap.series("delta_mocap_x_on_support_right"));
        plot.add("raw_delta_mocap_x_on_support_right", modelNoSensor.series("delta_mocap_x_on_support_right"));
        plot.plot("time", "all").render();
        plot.clear();
        
        plot.add("with_delta_mocap_y_on_support_left", modelWithMocap.series("delta_mocap_y_on_support_left"));
        plot.add("no_delta_head_y_on_support_left", modelNoMocap.series("delta_head_y_on_support_left"));
        plot.add("no_delta_mocap_y_on_support_left", modelNoMocap.series("delta_mocap_y_on_support_left"));
        plot.add("raw_delta_mocap_y_on_support_left", modelNoSensor.series("delta_mocap_y_on_support_left"));
        plot.plot("time", "all").render();
        plot.clear();
        plot.add("with_delta_mocap_y_on_support_right", modelWithMocap.series("delta_mocap_y_on_support_right"));
        plot.add("no_delta_head_y_on_support_right", modelNoMocap.series("delta_head_y_on_support_right"));
        plot.add("no_delta_mocap_y_on_support_right", modelNoMocap.series("delta_mocap_y_on_support_right"));
        plot.add("raw_delta_mocap_y_on_support_right", modelNoSensor.series("delta_mocap_y_on_support_right"));
        plot.plot("time", "all").render();
        plot.clear();
        
        plot.add("with_delta_mocap_theta_on_support_left", modelWithMocap.series("delta_mocap_theta_on_support_left"));
        plot.add("no_delta_head_theta_on_support_left", modelNoMocap.series("delta_head_theta_on_support_left"));
        plot.add("no_delta_mocap_theta_on_support_left", modelNoMocap.series("delta_mocap_theta_on_support_left"));
        plot.add("raw_delta_mocap_theta_on_support_left", modelNoSensor.series("delta_mocap_theta_on_support_left"));
        plot.plot("time", "all").render();
        plot.clear();
        plot.add("with_delta_mocap_theta_on_support_right", modelWithMocap.series("delta_mocap_theta_on_support_right"));
        plot.add("no_delta_head_theta_on_support_right", modelNoMocap.series("delta_head_theta_on_support_right"));
        plot.add("no_delta_mocap_theta_on_support_right", modelNoMocap.series("delta_mocap_theta_on_support_right"));
        plot.add("raw_delta_mocap_theta_on_support_right", modelNoSensor.series("delta_mocap_theta_on_support_right"));
        plot.plot("time", "all").render();
        plot.clear();
        
        plot.add("with_integrated_mocap_x", modelWithMocap.series("integrated_mocap_x"));
        plot.add("no_integrated_mocap_x", modelNoMocap.series("integrated_mocap_x"));
        plot.add("no_integrated_head_x", modelNoMocap.series("integrated_head_x"));
        plot.add("raw_integrated_mocap_x", modelNoSensor.series("integrated_mocap_x"));
        plot.plot("time", "all").render();
        plot.clear();
        
        plot.add("with_integrated_mocap_y", modelWithMocap.series("integrated_mocap_y"));
        plot.add("no_integrated_mocap_y", modelNoMocap.series("integrated_mocap_y"));
        plot.add("no_integrated_head_y", modelNoMocap.series("integrated_head_y"));
        plot.add("raw_integrated_mocap_y", modelNoSensor.series("integrated_mocap_y"));
        plot.plot("time", "all").render();
        plot.clear();
    }
 
    return 0;
}

