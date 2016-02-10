#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include <lwpr_eigen.hpp>
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Spline/Spline.hpp"
#include "Spline/SplineContainer.hpp"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "Plot/Plot.hpp"
#include "Utils/LWPRUtils.h"

/**
 * Learning task summary
 */
struct LearningTask {
    std::vector<std::string> logfiles;
    std::vector<std::string> inputs;
    std::vector<size_t> inputsLag;
    std::vector<std::string> outputs;
    std::function<void(Leph::MatrixLabel& logs)> funcComputeModelData;
    std::function<LWPR_Object(const LearningTask& task)> funcInitLWPR;
};

/**
 * Learning data used to proccess a learning task
 */
struct LearningData {
    std::vector<Leph::MatrixLabel> logs;
    std::vector<Eigen::VectorXd> inputsLearnData;
    std::vector<Eigen::VectorXd> outputsLearnData;
    std::vector<Eigen::VectorXd> inputsTestData;
    std::vector<Eigen::VectorXd> outputsTestData;
};

/**
 * Build inputs and outputs Vector at given logs index
 * Return empty vector if given index is not usable data
 */
Eigen::VectorXd learningRetrieveInputs(const LearningTask& task, const Leph::MatrixLabel& logs, size_t index)
{
    Eigen::VectorXd in(task.inputs.size());
    for (size_t i=0;i<task.inputs.size();i++) {
        size_t lag = task.inputsLag[i];
        if (index < lag || !logs[index-lag].exist(task.inputs[i])) {
            return Eigen::VectorXd();
        }
        in(i) = logs[index-lag](task.inputs[i]);
    }
    return in;
}
Eigen::VectorXd learningRetrieveOutputs(const LearningTask& task, const Leph::MatrixLabel& logs, size_t index)
{
    Eigen::VectorXd out(task.outputs.size());
    for (size_t i=0;i<task.outputs.size();i++) {
        if (!logs[index].exist(task.outputs[i])) {
            return Eigen::VectorXd();
        }
        out(i) = logs[index](task.outputs[i]);
    }
    return out;
}

/**
 * Init and return Learning Data
 */
LearningData learningInit(const LearningTask& task)
{
    LearningData data;
    //Load data from file and compute model
    for (size_t i=0;i<task.logfiles.size();i++) {
        data.logs.push_back(Leph::MatrixLabel());
        data.logs[i].load(task.logfiles[i]);
        task.funcComputeModelData(data.logs[i]);
    }
    //Compute learn and test data points
    size_t size = data.logs.size();
    if (size > 1) {
        //Split logs in two
        //Half rounded to upper integer
        size_t middle = (size + 1)/2 - 1;
        //Add learn data
        for (size_t k=0;k<=middle;k++) {
            for (size_t i=0;i<data.logs[k].size();i++) {
                Eigen::VectorXd in = learningRetrieveInputs(task, data.logs[k], i);
                Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs[k], i);
                if (in.size() > 0 && out.size() > 0) {
                    data.inputsLearnData.push_back(in);
                    data.outputsLearnData.push_back(out);
                }
            }
        }
        //Add test data
        for (size_t k=middle+1;k<size;k++) {
            for (size_t i=0;i<data.logs[k].size();i++) {
                Eigen::VectorXd in = learningRetrieveInputs(task, data.logs[k], i);
                Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs[k], i);
                if (in.size() > 0 && out.size() > 0) {
                    data.inputsTestData.push_back(in);
                    data.outputsTestData.push_back(out);
                }
            }
        }
    } else if (data.logs.size() == 1) {
        //Split the log in two sequence
        //Half rounded to upper integer
        size_t middle = (data.logs.front().size() + 1)/2 - 1;
        //Add learn data
        for (size_t i=0;i<=middle;i++) {
            Eigen::VectorXd in = learningRetrieveInputs(task, data.logs.front(), i);
            Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs.front(), i);
            if (in.size() > 0 && out.size() > 0) {
                data.inputsLearnData.push_back(in);
                data.outputsLearnData.push_back(out);
            }
        }
        //Add test data
        for (size_t i=middle+1;i<data.logs.front().size();i++) {
            Eigen::VectorXd in = learningRetrieveInputs(task, data.logs.front(), i);
            Eigen::VectorXd out = learningRetrieveOutputs(task, data.logs.front(), i);
            if (in.size() > 0 && out.size() > 0) {
                data.inputsTestData.push_back(in);
                data.outputsTestData.push_back(out);
            }
        }
    }

    return data;
}

void learningDoLearn(const LearningData& data, LWPR_Object& model)
{
    for (size_t i=0;i<data.inputsLearnData.size();i++) {
        model.update(data.inputsLearnData[i], data.outputsLearnData[i]);
    }
}

double learningDoTesting(const LearningData& data, LWPR_Object& model)
{
    double sumSquareError = 0.0;
    size_t count = 0;
    for (size_t i=0;i<data.inputsTestData.size();i++) {
        Eigen::VectorXd yp = model.predict(data.inputsTestData[i], 0.0);
        sumSquareError += (yp - data.outputsTestData[i]).squaredNorm();
        count++;
    }

    return sqrt(sumSquareError/count);
}
void learningTestStats(const LearningData& data, LWPR_Object& model)
{
    if (data.inputsTestData.size() == 0 || data.outputsTestData.size() == 0) {
        return;
    }
    size_t sizeIn = data.inputsTestData.front().size();
    size_t sizeOut = data.outputsTestData.front().size();
    Eigen::VectorXd sumSquareError = Eigen::VectorXd::Zero(sizeOut);
    Eigen::VectorXd maxError = Eigen::VectorXd::Constant(sizeOut, -1.0);
    Eigen::VectorXd sumMaxW = Eigen::VectorXd::Zero(sizeOut);
    Eigen::VectorXd minMaxW = Eigen::VectorXd::Constant(sizeOut, -1.0);
    Eigen::VectorXd sumInputs = Eigen::VectorXd::Zero(sizeIn);
    Eigen::VectorXd sumOutputs = Eigen::VectorXd::Zero(sizeOut);
    Eigen::VectorXd sumSquaredInputs = Eigen::VectorXd::Zero(sizeIn);
    Eigen::VectorXd sumSquaredOutputs = Eigen::VectorXd::Zero(sizeOut);
    size_t count = 0;
    for (size_t i=0;i<data.inputsTestData.size();i++) {
        Eigen::VectorXd confidence;
        Eigen::VectorXd maxW;
        Eigen::VectorXd yp = model.predict(data.inputsTestData[i], confidence, maxW, 0.0);
        Eigen::VectorXd error = yp - data.outputsTestData[i];
        sumSquareError += error.cwiseAbs2();
        for (size_t j=0;j<sizeOut;j++) {
            if (maxError(j) < 0.0 || maxError(j) < fabs(error(j))) {
                maxError(j) = fabs(error(j));
            }
            if (minMaxW(j) < 0.0 || minMaxW(j) > maxW(j)) {
                minMaxW(j) = maxW(j);
            }
        }
        sumMaxW += maxW;
        sumInputs += data.inputsTestData[i];
        sumOutputs += data.outputsTestData[i];
        sumSquaredInputs += data.inputsTestData[i].cwiseAbs2();
        sumSquaredOutputs += data.outputsTestData[i].cwiseAbs2();
        count++;
    }
    if (count == 0) {
        std::cout << "No Statistic" << std::endl;
        return;
    }
    sumSquareError /= count;
    sumMaxW /= count;
    std::cout << "Testing Stats:" << std::endl;
    std::cout << "Count=" << count << std::endl;
    std::cout << "MSE: " << sumSquareError.transpose() << std::endl;
    std::cout << "RMSE: " << sumSquareError.cwiseSqrt().transpose() << std::endl;
    std::cout << "MaxError: " << maxError.transpose() << std::endl;
    std::cout << "MeanMaxW: " << sumMaxW.transpose() << std::endl;
    std::cout << "MinMaxW: " << minMaxW.transpose() << std::endl;
    sumInputs /= count;
    sumOutputs /= count;
    std::cout << "Mean inputs: " << sumInputs.transpose() << std::endl;
    std::cout << "Mean outputs: " << sumOutputs.transpose() << std::endl;
    sumSquaredInputs /= count;
    sumSquaredOutputs /= count;
    sumSquaredInputs -= sumInputs.cwiseAbs2();
    sumSquaredOutputs -= sumOutputs.cwiseAbs2();
    std::cout << "StdDev inputs: " << sumSquaredInputs.cwiseSqrt().transpose() << std::endl;
    std::cout << "StdDev outputs: " << sumSquaredOutputs.cwiseSqrt().transpose() << std::endl;
    std::cout << "Var inputs: " << sumSquaredInputs.transpose() << std::endl;
    std::cout << "Var outputs: " << sumSquaredOutputs.transpose() << std::endl;
    Leph::LWPRPrint(model);
}

void learningOptimize(LearningTask& task)
{
    /*
    if (task.parameters.size() > 0) {
        //CMA-ES config
        libcmaes::CMAParameters<> cmaparams(task.parameters, 10.0);
        cmaparams.set_quiet(false);
        cmaparams.set_mt_feval(false);
        cmaparams.set_str_algo("acmaes");
        cmaparams.set_max_iter(20);
        //cmaparams.set_elitism(true);
        //Fitness function
        libcmaes::FitFuncEigen fitness = [&task]
            (const Eigen::VectorXd& params) 
            {
                task.parameters = params;
                double cost = task.funcBoundParameters(task.parameters);
                LWPR_Object model = task.funcInitLWPR(task);
                LearningData data = learningInit(task);
                learningDoLearn(data, model);
                cost += learningDoTesting(data, model);
                return cost;
            };
        //Do opimization
        libcmaes::CMASolutions cmasols = 
            libcmaes::cmaes<>(fitness, cmaparams);
        Eigen::VectorXd bestParams = 
            cmasols.get_best_seen_candidate().get_x_dvec();
        task.parameters = bestParams;
    }
    */
    for (size_t i=0;i<task.inputs.size();i++) {
        size_t bestLag = 0;
        double bestScore = -1.0;
        for (size_t k=0;k<20;k++) {
            task.inputsLag[i] = k;
            LWPR_Object model = task.funcInitLWPR(task);
            LearningData data = learningInit(task);
            learningDoLearn(data, model);
            double score = learningDoTesting(data, model);
            if (bestScore < 0.0 || bestScore > score) {
                bestScore = score;
                bestLag = k;
            }
            std::cout << "Optimizing " << task.inputs[i] << " lag=" << k << " score=" << score << std::endl;
        }
        task.inputsLag[i] = bestLag;
    }

    //Display found stats
    for (size_t i=0;i<task.inputs.size();i++) {
        std::cout << "Inputs: " << task.inputs[i] << " bestLag=" << task.inputsLag[i] << std::endl;
    }
    LearningData data = learningInit(task);
    LWPR_Object model = task.funcInitLWPR(task);
    learningDoLearn(data, model);
    learningTestStats(data, model);
}

static std::vector<std::string> allDOFs = {
    "left_ankle_pitch", "left_ankle_roll", "left_knee",
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
    "right_ankle_pitch", "right_ankle_roll", "right_knee",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw"
};
static std::vector<std::string> leftDOFs = {
    "left_ankle_pitch", "left_ankle_roll", "left_knee",
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw"
};
static std::vector<std::string> rightDOFs = {
    "right_ankle_pitch", "right_ankle_roll", "right_knee",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw"
};
static std::vector<std::string> leftHipDOFs = {
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw"
};
static std::vector<std::string> rightHipDOFs = {
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw"
};
static std::vector<std::string> leftAnkleDOFs = {
    "left_ankle_pitch", "left_ankle_roll", "left_knee"
};
static std::vector<std::string> rightAnkleDOFs = {
    "right_ankle_pitch", "right_ankle_roll", "right_knee"
};
static std::vector<std::string> allIKDOFs = {
    "trunk_pos_x", "trunk_pos_y", "trunk_pos_z",
    "trunk_axis_x", "trunk_axis_y", "trunk_axis_z",
    "foot_pos_x", "foot_pos_y", "foot_pos_z",
    "foot_axis_x", "foot_axis_y", "foot_axis_z"
};
static std::vector<std::string> trunkPosIKDOFs = {
    "trunk_pos_x", "trunk_pos_y", "trunk_pos_z",
};
static std::vector<std::string> trunkAxisIKDOFs = {
    "trunk_axis_x", "trunk_axis_y", "trunk_axis_z",
};
static std::vector<std::string> footPosIKDOFs = {
    "foot_pos_x", "foot_pos_y", "foot_pos_z",
};
static std::vector<std::string> footAxisIKDOFs = {
    "foot_axis_x", "foot_axis_y", "foot_axis_z"
};

/**
 * Compute and insert in given logs container
 * model data and goal trajectory from splines
 */
void computeModelData(Leph::MatrixLabel& logs, const Leph::SplineContainer<Leph::Spline> splines)
{
    Leph::HumanoidFixedModel modelSpline(Leph::SigmabanModel);
    Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);
    Leph::HumanoidFixedPressureModel modelPos(Leph::SigmabanModel);
    for (size_t index=1;index<logs.size();index++) {
        double t = logs[index-1]("t");
        //Compute spline state
        Eigen::Vector3d trunkPos;
        Eigen::Vector3d trunkAxis;
        Eigen::Vector3d footPos;
        Eigen::Vector3d footAxis;
        trunkPos.x() = splines.get("trunk_pos_x").pos(t);
        trunkPos.y() = splines.get("trunk_pos_y").pos(t);
        trunkPos.z() = splines.get("trunk_pos_z").pos(t);
        trunkAxis.x() = splines.get("trunk_axis_x").pos(t);
        trunkAxis.y() = splines.get("trunk_axis_y").pos(t);
        trunkAxis.z() = splines.get("trunk_axis_z").pos(t);
        footPos.x() = splines.get("foot_pos_x").pos(t);
        footPos.y() = splines.get("foot_pos_y").pos(t);
        footPos.z() = splines.get("foot_pos_z").pos(t);
        footAxis.x() = splines.get("foot_axis_x").pos(t);
        footAxis.y() = splines.get("foot_axis_y").pos(t);
        footAxis.z() = splines.get("foot_axis_z").pos(t);
        //Compute goal DOF
        bool isSuccess = modelSpline.trunkFootIK(
            Leph::HumanoidFixedModel::LeftSupportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAxis),
            footPos,
            Leph::AxisToMatrix(footAxis));
        if (!isSuccess) {
            std::cout << "IK ERROR" << std::endl;
            exit(1);
        }
        for (const std::string& name : allDOFs) {
            logs[index].append("spline_pos:" + name, modelSpline.get().getDOF(name));
        }
        //Insert splines
        for (const std::string& name : allIKDOFs) {
            logs[index].append("spline_model:" + name, splines.get(name).pos(t));
        }
        //Assign DOF
        Leph::VectorLabel vectDOFGoal = 
            logs[index].extract("goal").rename("goal", "");
        modelGoal.get().setDOF(vectDOFGoal, false);
        Leph::VectorLabel vectDOFPos = 
            logs[index].extract("pos").rename("pos", "");
        modelPos.get().setDOF(vectDOFPos, false);
        //Assiging pressure
        modelPos.setPressure(
            logs[index]("pressure:weight"),
            logs[index]("pressure:left_ratio"),
            logs[index]("pressure:right_ratio"),
            logs[index]("pressure:left_x"),
            logs[index]("pressure:left_y"),
            logs[index]("pressure:right_x"),
            logs[index]("pressure:right_y"));
        //Contraint the model on the ground
        modelGoal.updateBase();
        modelPos.updateBase();
        //Set IMU data for motors real model state
        modelPos.setOrientation(
            logs[index]("sensor:pitch"), 
            logs[index]("sensor:roll"));
        //Compute model goal
        logs[index].append("goal_model:trunk_pos_x", modelGoal.get().position("trunk", "left_foot_tip").x());
        logs[index].append("goal_model:trunk_pos_y", modelGoal.get().position("trunk", "left_foot_tip").y());
        logs[index].append("goal_model:trunk_pos_z", modelGoal.get().position("trunk", "left_foot_tip").z());
        logs[index].append("goal_model:foot_pos_x", modelGoal.get().position("right_foot_tip", "left_foot_tip").x());
        logs[index].append("goal_model:foot_pos_y", modelGoal.get().position("right_foot_tip", "left_foot_tip").y());
        logs[index].append("goal_model:foot_pos_z", modelGoal.get().position("right_foot_tip", "left_foot_tip").z());
        logs[index].append("goal_model:trunk_axis_x", Leph::MatrixToAxis(modelGoal.get().orientation("trunk", "left_foot_tip").transpose()).x());
        logs[index].append("goal_model:trunk_axis_y", Leph::MatrixToAxis(modelGoal.get().orientation("trunk", "left_foot_tip").transpose()).y());
        logs[index].append("goal_model:trunk_axis_z", Leph::MatrixToAxis(modelGoal.get().orientation("trunk", "left_foot_tip").transpose()).z());
        logs[index].append("goal_model:foot_axis_x", Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", "left_foot_tip").transpose()).x());
        logs[index].append("goal_model:foot_axis_y", Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", "left_foot_tip").transpose()).y());
        logs[index].append("goal_model:foot_axis_z", Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", "left_foot_tip").transpose()).z());
        //Compute model pos
        logs[index].append("model:trunk_pos_x", modelPos.get().position("trunk", "left_foot_tip").x());
        logs[index].append("model:trunk_pos_y", modelPos.get().position("trunk", "left_foot_tip").y());
        logs[index].append("model:trunk_pos_z", modelPos.get().position("trunk", "left_foot_tip").z());
        logs[index].append("model:foot_pos_x", modelPos.get().position("right_foot_tip", "left_foot_tip").x());
        logs[index].append("model:foot_pos_y", modelPos.get().position("right_foot_tip", "left_foot_tip").y());
        logs[index].append("model:foot_pos_z", modelPos.get().position("right_foot_tip", "left_foot_tip").z());
        logs[index].append("model:trunk_axis_x", Leph::MatrixToAxis(modelPos.get().orientation("trunk", "left_foot_tip").transpose()).x());
        logs[index].append("model:trunk_axis_y", Leph::MatrixToAxis(modelPos.get().orientation("trunk", "left_foot_tip").transpose()).y());
        logs[index].append("model:trunk_axis_z", Leph::MatrixToAxis(modelPos.get().orientation("trunk", "left_foot_tip").transpose()).z());
        logs[index].append("model:foot_axis_x", Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", "left_foot_tip").transpose()).x());
        logs[index].append("model:foot_axis_y", Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", "left_foot_tip").transpose()).y());
        logs[index].append("model:foot_axis_z", Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", "left_foot_tip").transpose()).z());
    }
}

void testLearning()
{
    std::string splinesfile = "../../These/Data/logs-2016-02-02/trajBest_3.000000_1.000000_0.000000_2.398569.splines";
    Leph::SplineContainer<Leph::Spline> splines;
    splines.importData(splinesfile);
    
    LearningTask task;
    task.logfiles = {
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-24-53.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-17.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-41.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-26-04.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-01.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-26.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-49.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-26-12.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-10.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-34.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-56.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-26-19.log",
    };
    task.inputs = {"model:trunk_pos_x", "goal_model:trunk_pos_x"};
    task.inputsLag = {0, 0};
    task.outputs = {"pressure:left_x"};
    task.funcComputeModelData = [&splines](Leph::MatrixLabel& logs){
        computeModelData(logs, splines);
    };
    std::function<void(Leph::MatrixLabel& logs)> funcComputeModelData;
    task.funcInitLWPR = [](const LearningTask& task) -> LWPR_Object {
        LWPR_Object model(task.inputs.size(), task.outputs.size());
        //Normalisation value for each input dimension (>0)
        model.normIn(0.01);
        //Use only diagonal matrix. Big speed up in high dimension
        //but lower performance in complex learning.
        model.diagOnly(false);
        //Automatic tunning of distance metric 
        model.useMeta(true);
        model.updateD(true);
        //Larger value enforce wider receptive field (>0)
        model.penalty(1e-6);
        //Set diagonal (input_dim) or complet (input_dim*input_dim)
        //initial distance matrix (>0)
        model.setInitD(3.0);
        return model;
    };

    learningOptimize(task);
}

int main()
{
    testLearning();
    //return 0;

    //Trajectory splines
    std::string splinesfile = "../../These/Data/logs-2016-02-02/trajBest_3.000000_1.000000_0.000000_2.398569.splines";
    //Loading splines
    Leph::SplineContainer<Leph::Spline> splines;
    splines.importData(splinesfile);
    
    //Loading data
    Leph::MatrixLabel logs;
    std::vector<std::string> allLogsFiles = {
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-24-53.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-17.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-41.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-26-04.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-01.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-26.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-49.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-26-12.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-10.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-34.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-25-56.log",
        "../../These/Data/logs-2016-02-02/tmpTrajLog3/model_2016-02-08-13-26-19.log",
    };
    for (const std::string& name : allLogsFiles) {
        logs.load(name);
    }

    //Compute model data
    computeModelData(logs, splines);
    
    Leph::Plot plot = logs.plot();
    
    plot
        .plot("goal_model:trunk_pos_x", "model:trunk_pos_x")
        .render();
    plot
        .plot("t", "goal_model:trunk_pos_x")
        .plot("t", "model:trunk_pos_x")
        .render();
    
    for (const std::string& name : trunkPosIKDOFs) {
        plot.plot("t", "goal_model:" + name);
    }
    plot.render();
    for (const std::string& name : trunkPosIKDOFs) {
        plot.plot("t", "goal_model:" + name);
        plot.plot("t", "model:" + name);
    }
    plot.render();
    plot
        .plot("t", "pressure:left_x")
        .plot("t", "pressure:left_y")
        .render();
    plot
        .plot("t", "model:trunk_pos_x")
        .plot("t", "goal_model:trunk_pos_x")
        .plot("t", "pressure:left_x")
        .render();
    plot
        .plot("t", "model:trunk_pos_x")
        .plot("t", "model:foot_pos_x")
        .plot("t", "pressure:left_x")
        .render();
    plot
        .plot("model:trunk_pos_x", "model:foot_pos_x", "pressure:left_x")
        .render();
    plot
        .plot("goal_model:trunk_pos_x", "model:trunk_pos_x", "pressure:left_x")
        .render();
    plot
        .plot("goal_model:trunk_pos_y", "goal_model:trunk_pos_x", "model:trunk_pos_x")
        .render();

    
    for (const std::string& name : trunkPosIKDOFs) {
        plot.plot("t", "goal_model:"+name);
        plot.plot("t", "model:"+name);
    }
    plot.render();
    for (const std::string& name : trunkAxisIKDOFs) {
        plot.plot("t", "goal_model:"+name);
        plot.plot("t", "model:"+name);
    }
    plot.render();
    for (const std::string& name : footPosIKDOFs) {
        plot.plot("t", "goal_model:"+name);
        plot.plot("t", "model:"+name);
    }
    plot.render();
    for (const std::string& name : footAxisIKDOFs) {
        plot.plot("t", "goal_model:"+name);
        plot.plot("t", "model:"+name);
    }
    plot.render();

    return 0;
    
    //Initialize model instances
    Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);
    Leph::HumanoidFixedPressureModel model(Leph::SigmabanModel);
    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling(50.0);
    
    //Main viewer loop
    size_t indexLog = 0;
    Leph::VectorLabel vectDOF = 
        logs[0].extract("pos").rename("pos", "");
    while (viewer.update()) {
        //Assign DOF
        Leph::VectorLabel vectDOF = 
            logs[indexLog].extract("pos").rename("pos", "");
        model.get().setDOF(vectDOF, false);
        //Assign splines goals
        Leph::VectorLabel vectDOFGoal = 
            logs[indexLog].extract("spline_pos").rename("spline_pos", "");
        modelGoal.get().setDOF(vectDOFGoal, false);
        //Assiging pressure
        model.setPressure(
            logs[indexLog]("pressure:weight"),
            logs[indexLog]("pressure:left_ratio"),
            logs[indexLog]("pressure:right_ratio"),
            logs[indexLog]("pressure:left_x"),
            logs[indexLog]("pressure:left_y"),
            logs[indexLog]("pressure:right_x"),
            logs[indexLog]("pressure:right_y"));
        //Contraint the model on the ground
        model.updateBase();
        //Set IMU data for motors real model state
        model.setOrientation(
            logs[indexLog]("sensor:pitch"), 
            logs[indexLog]("sensor:roll"));
        //Retrieve center of pressure
        Eigen::Vector3d copLeft = model.centerOfPressureLeft("origin");
        copLeft.z() = 0.0;
        //Display foot pressure force
        viewer.drawBox(0.005, 0.005, 
            0.1*model.pressureLeftRatio(),
            copLeft + Eigen::Vector3d(
                0, 0, 0.1*model.pressureLeftRatio()), 
            Eigen::Matrix3d::Identity(),
            1.0, 0.0, 0.0);
        //Display pressure trajectory
        viewer.addTrackedPoint(
            copLeft, Leph::ModelViewer::Red);
        //Display
        Leph::ModelDraw(model.get(), viewer);
        Leph::ModelDraw(modelGoal.get(), viewer);
        //Scheduling
        scheduling.wait();
        //Index
        indexLog++;
        if (indexLog >= logs.size()) {
            indexLog = 0;
        }
    }

    return 0;
}

