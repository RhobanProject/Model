#include "TimeSeries/SeriesUtils.h"
#include "Concepts/HumanoidModelConcept.hpp"
#include "Concepts/FootStepDifferentiatorConcept.hpp"
#include "Concepts/FootStepIntegratorConcept.hpp"
#include "Concepts/FallDetectorConcept.hpp"

namespace Leph {

void plotPhase(
    Leph::Plot& plot,
    Leph::ModelSeries& model, 
    const std::string& name1, 
    const std::vector<std::string>& names)
{
    if (model.series(name1).size() == 0) {
        return;
    }
    for (size_t i=0;i<names.size();i++) {
        if (model.series(names[i]).size() == 0) {
            return;
        }
    }

    double min = model.series(name1).timeMin();
    double max = model.series(name1).timeMax();
    for (size_t i=0;i<names.size();i++) {
        if (model.series(names[i]).timeMin() > min) {
            min = model.series(names[i]).timeMin();
        }
        if (model.series(names[i]).timeMax() < max) {
            max = model.series(names[i]).timeMax();
        }
    }

    size_t indexMin = model.series(name1).getClosestIndex(min);
    size_t indexMax = model.series(name1).getClosestIndex(max);
    for (size_t i=indexMax;i<=indexMin;i++) {
        bool isPoint = true;
        double t = model.series(name1).at(i).time;
        double val1 = model.series(name1).at(i).value;
        Leph::VectorLabel vect;
        vect.append("time", t);
        vect.append(name1, val1);
        for (size_t i=0;i<names.size();i++) {
            if (model.series(names[i]).isTimeValid(t)) {
                double val2 = model.series(names[i]).get(t);
                vect.append(names[i], val2);
            } else {
                isPoint = false;
            }
        }
        if (isPoint) {
            plot.add(vect);
        }
    }
}

void plotFuture(
    Leph::Plot& plot,
    Leph::ModelSeries& model, 
    const std::string& name)
{
    if (
        model.series(name).size() == 0 ||
        model.series(name).sizeFuture() == 0
    ) {
        return;
    }

    for (size_t i=0;i<model.series(name).sizeFuture();i++) {
        double time = model.series(name).atFuture(i).time;
        if (model.series(name).isTimeValid(time)) {
            plot.add(Leph::VectorLabel(
                "time", time,
                name + "_future", model.series(name).atFuture(i).value,
                name, model.series(name).get(time)));
        }
    }
}

double modelPredict(
    Leph::ModelSeries& model, 
    const std::string& name,
    const std::vector<double>& inputs)
{
    Eigen::VectorXd vect(inputs.size());
    for (size_t i=0;i<inputs.size();i++) {
        vect(i) = inputs[i];
    }

    return model.regression(name)
        .model().predict(vect, 0.0)(0);
}

double seriesDistance(
    const Leph::TimeSeries& series1, 
    const Leph::TimeSeries& series2)
{
    if (series1.size() < 2 ||
        series2.size() < 2
    ) {
        return -1.0;
    }

    double min = series1.timeMin();
    if (min < series2.timeMin()) {
        min = series2.timeMin();
    }
    double max = series1.timeMax();
    if (max > series2.timeMax()) {
        max = series2.timeMax();
    }

    double time = min;
    double sumSquared = 0.0;
    int count = 0;
    while (time < max) {
        size_t index1 = series1.getLowerIndex(time);
        size_t index2 = series2.getLowerIndex(time);
        double t1 = series1.at(index1).time;
        double t2 = series2.at(index2).time;
        double t = (t1 > t1) ? t1 : t2;
        double val1 = series1.get(t);
        double val2 = series2.get(t);
        sumSquared += pow(val1-val2, 2);
        count++;
        time = t;
        time += Leph::TIME_EPSILON;
    }

    if (count == 0) {
        return -1.0;
    } else {
        return sqrt(sumSquared/count);
    }
}

void initModelSeries(Leph::ModelSeries& model, 
    bool withMocapConcept,
    bool withDeltaRegression,
    bool withWalkRegression)
{
    //Declare TimeSeries
    //Inputs DOF
    model.addSeries("left_ankle_roll");
    model.addSeries("left_ankle_pitch");
    model.addSeries("left_knee");
    model.addSeries("left_hip_pitch");
    model.addSeries("left_hip_roll");
    model.addSeries("left_hip_yaw");
    model.addSeries("right_ankle_roll");
    model.addSeries("right_ankle_pitch");
    model.addSeries("right_knee");
    model.addSeries("right_hip_pitch");
    model.addSeries("right_hip_roll");
    model.addSeries("right_hip_yaw");
    model.addSeries("left_shoulder_pitch");
    model.addSeries("left_shoulder_roll");
    model.addSeries("left_elbow");
    model.addSeries("right_shoulder_pitch");
    model.addSeries("right_shoulder_roll");
    model.addSeries("right_elbow");
    model.addSeries("head_yaw");
    model.addSeries("head_pitch");
    //Inputs sensors
    model.addSeries("sensor_pitch");
    model.addSeries("sensor_roll");
    model.addSeries("sensor_gyro_yaw");
    model.addSeries("sensor_gyro_x");
    model.addSeries("sensor_gyro_y");
    model.addSeries("sensor_acc_x");
    model.addSeries("sensor_acc_y");
    //Inputs pressure
    model.addSeries("pressure_weight");
    model.addSeries("pressure_left_ratio");
    model.addSeries("pressure_right_ratio");
    model.addSeries("pressure_left_x");
    model.addSeries("pressure_left_y");
    model.addSeries("pressure_right_x");
    model.addSeries("pressure_right_y");
    //Inputs Mocap
    model.addSeries("mocap_is_valid");
    model.addSeries("mocap_x");
    model.addSeries("mocap_y");
    model.addSeries("mocap_theta");
    //Humanoid model outputs
    model.addSeries("is_support_foot_left");
    model.addSeries("head_x");
    model.addSeries("head_y");
    model.addSeries("head_z");
    model.addSeries("head_theta");
    model.addSeries("left_foot_x");
    model.addSeries("left_foot_y");
    model.addSeries("left_foot_z");
    model.addSeries("left_foot_theta");
    model.addSeries("right_foot_x");
    model.addSeries("right_foot_y");
    model.addSeries("right_foot_z");
    model.addSeries("right_foot_theta");
    //FootStep Differentiator outputs
    //head
    model.addSeries("delta_head_x_on_support_left");
    model.addSeries("delta_head_y_on_support_left");
    model.addSeries("delta_head_theta_on_support_left");
    model.addSeries("delta_head_x_on_support_right");
    model.addSeries("delta_head_y_on_support_right");
    model.addSeries("delta_head_theta_on_support_right");
    //Left foot
    model.addSeries("delta_left_foot_x_on_support_left");
    model.addSeries("delta_left_foot_y_on_support_left");
    model.addSeries("delta_left_foot_theta_on_support_left");
    model.addSeries("delta_left_foot_x_on_support_right");
    model.addSeries("delta_left_foot_y_on_support_right");
    model.addSeries("delta_left_foot_theta_on_support_right");
    //Right foot
    model.addSeries("delta_right_foot_x_on_support_left");
    model.addSeries("delta_right_foot_y_on_support_left");
    model.addSeries("delta_right_foot_theta_on_support_left");
    model.addSeries("delta_right_foot_x_on_support_right");
    model.addSeries("delta_right_foot_y_on_support_right");
    model.addSeries("delta_right_foot_theta_on_support_right");
    //Mocap
    model.addSeries("delta_mocap_x_on_support_left");
    model.addSeries("delta_mocap_y_on_support_left");
    model.addSeries("delta_mocap_theta_on_support_left");
    model.addSeries("delta_mocap_x_on_support_right");
    model.addSeries("delta_mocap_y_on_support_right");
    model.addSeries("delta_mocap_theta_on_support_right");
    //Walk orders reference
    model.addSeries("walk_step");
    model.addSeries("walk_lateral");
    model.addSeries("walk_turn");
    model.addSeries("walk_enabled");
    //Fall Detector output
    model.addSeries("is_fallen");
    //FootStep Integrator output
    //head
    model.addSeries("integrated_head_x");
    model.addSeries("integrated_head_y");
    model.addSeries("integrated_head_theta");
    //Mocap
    model.addSeries("integrated_mocap_x");
    model.addSeries("integrated_mocap_y");
    model.addSeries("integrated_mocap_theta");

    //Initialize Model Concept
    model.addConcept(
        //HumanoidModelConcept allocation
        new Leph::HumanoidModelConcept(Leph::SigmabanModel), 
        //Inputs
        {"left_ankle_roll",
        "left_ankle_pitch",
        "left_knee",
        "left_hip_pitch",
        "left_hip_roll",
        "left_hip_yaw",
        "right_ankle_roll",
        "right_ankle_pitch",
        "right_knee",
        "right_hip_pitch",
        "right_hip_roll",
        "right_hip_yaw",
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_elbow",
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_elbow",
        "head_yaw",
        "head_pitch",
        "sensor_pitch",
        "sensor_roll",
        "sensor_gyro_yaw",
        "pressure_weight",
        "pressure_left_ratio", 
        "pressure_right_ratio",
        "pressure_left_x",
        "pressure_left_y",
        "pressure_right_x",
        "pressure_right_y",
        "is_fallen"},
        //Outputs
        {"is_support_foot_left",
        "head_x",
        "head_y",
        "head_z",
        "head_theta",
        "left_foot_x",
        "left_foot_y",
        "left_foot_z",
        "left_foot_theta",
        "right_foot_x",
        "right_foot_y",
        "right_foot_z",
        "right_foot_theta"});
    //Fall Detector concept
    model.addConcept(
        new Leph::FallDetectorConcept(),
        //Inputs
        {"sensor_pitch",
        "sensor_roll"},
        //Output
        {"is_fallen"});
    //Differentiation footstep concept
    model.addConcept(
        //FootStepDifferentiatorConcept allocation
        new Leph::FootStepDifferentiatorConcept(),
        //Inputs
        {"is_support_foot_left",
        "head_x",
        "head_y",
        "head_theta"},
        //Outputs
        {"delta_head_x_on_support_left",
        "delta_head_y_on_support_left",
        "delta_head_theta_on_support_left",
        "delta_head_x_on_support_right",
        "delta_head_y_on_support_right",
        "delta_head_theta_on_support_right"});
    model.addConcept(
        //FootStepDifferentiatorConcept allocation
        new Leph::FootStepDifferentiatorConcept(),
        //Inputs
        {"is_support_foot_left",
        "left_foot_x",
        "left_foot_y",
        "left_foot_theta"},
        //Outputs
        {"delta_left_foot_x_on_support_left",
        "delta_left_foot_y_on_support_left",
        "delta_left_foot_theta_on_support_left",
        "delta_left_foot_x_on_support_right",
        "delta_left_foot_y_on_support_right",
        "delta_left_foot_theta_on_support_right"});
    model.addConcept(
        //FootStepDifferentiatorConcept allocation
        new Leph::FootStepDifferentiatorConcept(),
        //Inputs
        {"is_support_foot_left",
        "right_foot_x",
        "right_foot_y",
        "right_foot_theta"},
        //Outputs
        {"delta_right_foot_x_on_support_left",
        "delta_right_foot_y_on_support_left",
        "delta_right_foot_theta_on_support_left",
        "delta_right_foot_x_on_support_right",
        "delta_right_foot_y_on_support_right",
        "delta_right_foot_theta_on_support_right"});
    if (withMocapConcept) {
        model.addConcept(
            //FootStepDifferentiatorConcept allocation
            new Leph::FootStepDifferentiatorConcept(),
            //Inputs
            {"is_support_foot_left",
            "mocap_x",
            "mocap_y",
            "mocap_theta"},
            //Outputs
            {"delta_mocap_x_on_support_left",
            "delta_mocap_y_on_support_left",
            "delta_mocap_theta_on_support_left",
            "delta_mocap_x_on_support_right",
            "delta_mocap_y_on_support_right",
            "delta_mocap_theta_on_support_right"});
    }
    model.addConcept(
        //FootStepIntegratorConcept allocation
        new Leph::FootStepIntegratorConcept(),
        //Inputs
        {"delta_head_x_on_support_left",
        "delta_head_y_on_support_left",
        "delta_head_theta_on_support_left",
        "delta_head_x_on_support_right",
        "delta_head_y_on_support_right",
        "delta_head_theta_on_support_right"},
        //Outputs
        {"integrated_head_x",
        "integrated_head_y",
        "integrated_head_theta"});
    model.addConcept(
        //FootStepIntegratorConcept allocation
        new Leph::FootStepIntegratorConcept(),
        //Inputs
        {"delta_mocap_x_on_support_left",
        "delta_mocap_y_on_support_left",
        "delta_mocap_theta_on_support_left",
        "delta_mocap_x_on_support_right",
        "delta_mocap_y_on_support_right",
        "delta_mocap_theta_on_support_right"},
        //Outputs
        {"integrated_mocap_x",
        "integrated_mocap_y",
        "integrated_mocap_theta"});

    //Initialize Regression model for 
    //mocap deltas
    auto funcAddRegressionDeltaLeft = [](Leph::ModelSeries& model, 
        const std::string& regressionName, const std::string& seriesName)
    {
        model.addRegression(regressionName, seriesName);
        model.regressionAddInput(regressionName, "delta_head_x_on_support_left");
        model.regressionAddInput(regressionName, "delta_head_y_on_support_left");
        model.regressionAddInput(regressionName, "delta_head_theta_on_support_left");
        model.regressionAddInput(regressionName, "delta_head_x_on_support_left", 1.0);
        model.regressionAddInput(regressionName, "delta_head_y_on_support_left", 1.0);
        model.regressionAddInput(regressionName, "delta_head_theta_on_support_left", 1.0);
        model.regressionAddInput(regressionName, "pressure_left_x", 1.0);
        model.regressionAddInput(regressionName, "pressure_left_y", 1.0);
        model.regressionAddInput(regressionName, "pressure_left_ratio", 1.0);
    };
    auto funcAddRegressionDeltaRight = [](Leph::ModelSeries& model, 
        const std::string& regressionName, const std::string& seriesName)
    {
        model.addRegression(regressionName, seriesName);
        model.regressionAddInput(regressionName, "delta_head_x_on_support_right");
        model.regressionAddInput(regressionName, "delta_head_y_on_support_right");
        model.regressionAddInput(regressionName, "delta_head_theta_on_support_right");
        model.regressionAddInput(regressionName, "delta_head_x_on_support_right", 1.0);
        model.regressionAddInput(regressionName, "delta_head_y_on_support_right", 1.0);
        model.regressionAddInput(regressionName, "delta_head_theta_on_support_right", 1.0);
        model.regressionAddInput(regressionName, "pressure_right_x", 1.0);
        model.regressionAddInput(regressionName, "pressure_right_y", 1.0);
        model.regressionAddInput(regressionName, "pressure_right_ratio", 1.0);
    };
    if (withDeltaRegression) {
        funcAddRegressionDeltaLeft(model, 
            "model_delta_mocap_x_left", "delta_mocap_x_on_support_left");
        funcAddRegressionDeltaLeft(model, 
            "model_delta_mocap_y_left", "delta_mocap_y_on_support_left");
        funcAddRegressionDeltaLeft(model, 
            "model_delta_mocap_theta_left", "delta_mocap_theta_on_support_left");
        funcAddRegressionDeltaRight(model, 
            "model_delta_mocap_x_right", "delta_mocap_x_on_support_right");
        funcAddRegressionDeltaRight(model, 
            "model_delta_mocap_y_right", "delta_mocap_y_on_support_right");
        funcAddRegressionDeltaRight(model, 
            "model_delta_mocap_theta_right", "delta_mocap_theta_on_support_right");
    }

    //Initialize regression model for 
    //delta step from walk orders
    auto funcAddRegressionWalk = [](Leph::ModelSeries& model, 
        const std::string& regressionName, const std::string& seriesName)
    {
        model.addRegression(regressionName, seriesName);
        model.regressionAddInput(regressionName, "walk_enabled");
        model.regressionAddInput(regressionName, "walk_step");
        model.regressionAddInput(regressionName, "walk_lateral");
        model.regressionAddInput(regressionName, "walk_turn");
        model.regressionAddInput(regressionName, "walk_step", 1.0);
        model.regressionAddInput(regressionName, "walk_lateral", 1.0);
        model.regressionAddInput(regressionName, "walk_turn", 1.0);
        model.regressionAddInput(regressionName, "walk_step", 2.0);
        model.regressionAddInput(regressionName, "walk_lateral", 2.0);
        model.regressionAddInput(regressionName, "walk_turn", 2.0);
    };
    if (withWalkRegression) {
        funcAddRegressionWalk(model, 
            "model_direct_delta_x_left", "delta_mocap_x_on_support_left");
        funcAddRegressionWalk(model, 
            "model_direct_delta_y_left", "delta_mocap_y_on_support_left");
        funcAddRegressionWalk(model, 
            "model_direct_delta_theta_left", "delta_mocap_theta_on_support_left");
        funcAddRegressionWalk(model, 
            "model_direct_delta_x_right", "delta_mocap_x_on_support_right");
        funcAddRegressionWalk(model, 
            "model_direct_delta_y_right", "delta_mocap_y_on_support_right");
        funcAddRegressionWalk(model, 
            "model_direct_delta_theta_right", "delta_mocap_theta_on_support_right");
    }
}

void appendModelSeries(
    Leph::ModelSeries& model, 
    double time, 
    const Leph::VectorLabel& logs)
{
    static double initGyroYaw = logs("sensor:gyro_yaw");
    static double initMocapX = logs("mocap:z");
    static double initMocapY = logs("mocap:x");
    static double initMocapTheta = logs("mocap:azimuth");
    //Loading low level inputs
    //Degrees of freedom
    model.series("left_ankle_roll").append(time, 
        logs("pos:left_ankle_roll"));
    model.series("left_ankle_pitch").append(time, 
        logs("pos:left_ankle_pitch"));
    model.series("left_knee").append(time, 
        logs("pos:left_knee"));
    model.series("left_hip_pitch").append(time, 
        logs("pos:left_hip_pitch"));
    model.series("left_hip_roll").append(time, 
        logs("pos:left_hip_roll"));
    model.series("left_hip_yaw").append(time, 
        logs("pos:left_hip_yaw"));
    model.series("right_ankle_roll").append(time, 
        logs("pos:right_ankle_roll"));
    model.series("right_ankle_pitch").append(time, 
        logs("pos:right_ankle_pitch"));
    model.series("right_knee").append(time, 
        logs("pos:right_knee"));
    model.series("right_hip_pitch").append(time, 
        logs("pos:right_hip_pitch"));
    model.series("right_hip_roll").append(time, 
        logs("pos:right_hip_roll"));
    model.series("right_hip_yaw").append(time, 
        logs("pos:right_hip_yaw"));
    model.series("left_shoulder_roll").append(time, 
        logs("pos:left_shoulder_roll"));
    model.series("left_shoulder_pitch").append(time, 
        logs("pos:left_shoulder_pitch"));
    model.series("left_elbow").append(time, 
        logs("pos:left_elbow"));
    model.series("right_shoulder_roll").append(time, 
        logs("pos:right_shoulder_roll"));
    model.series("right_shoulder_pitch").append(time, 
        logs("pos:right_shoulder_pitch"));
    model.series("right_elbow").append(time, 
        logs("pos:right_elbow"));
    model.series("head_yaw").append(time, 
        logs("pos:head_yaw"));
    model.series("head_pitch").append(time, 
        logs("pos:head_pitch"));
    //Sensors
    model.series("sensor_pitch").append(time, 
        logs("sensor:pitch"));
    model.series("sensor_roll").append(time, 
        logs("sensor:roll"));
    model.series("sensor_gyro_yaw").append(time, 
        logs("sensor:gyro_yaw") - initGyroYaw);
    model.series("sensor_gyro_x").append(time, 
        logs("sensor:gyro_x"));
    model.series("sensor_gyro_y").append(time, 
        logs("sensor:gyro_y"));
    model.series("sensor_acc_x").append(time, 
        logs("sensor:acc_x"));
    model.series("sensor_acc_y").append(time, 
        logs("sensor:acc_y"));
    //Pressure
    model.series("pressure_weight").append(time, 
        logs("pressure:weight"));
    model.series("pressure_left_ratio").append(time, 
        logs("pressure:left_ratio"));
    model.series("pressure_right_ratio").append(time, 
        logs("pressure:right_ratio"));
    model.series("pressure_left_x").append(time, 
        logs("pressure:left_x"));
    model.series("pressure_left_y").append(time, 
        logs("pressure:left_y"));
    model.series("pressure_right_x").append(time, 
        logs("pressure:right_x"));
    model.series("pressure_right_y").append(time, 
        logs("pressure:right_y"));
    //Walk orders
    model.series("walk_step").append(time,
        logs("walk:stepGain"));
    model.series("walk_lateral").append(time,
        logs("walk:lateralGain"));
    model.series("walk_turn").append(time,
        logs("walk:turnGain"));
    model.series("walk_enabled").append(time,
        logs("walk:enabledGain"));
    //Motion Capture
    bool isMocapValid = logs("mocap:is_valid");
    if (isMocapValid) {
        model.series("mocap_x").append(time,
            (logs("mocap:z") - initMocapX));
        model.series("mocap_y").append(time,
            (logs("mocap:x") - initMocapY));
        double mocapTheta = (logs("mocap:azimuth") 
            - initMocapTheta)*M_PI/180.0;
        model.series("mocap_theta").append(time, mocapTheta);
    }
    model.series("mocap_is_valid").append(time,
        isMocapValid);
}

}

