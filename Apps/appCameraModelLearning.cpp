#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include "Calibration/LogLikelihoodMaximization.hpp"
#include "Types/MatrixLabel.hpp"
#include "Model/HumanoidModel.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/JointModel.hpp"
#include "Model/NamesModel.h"
#include "Utils/FileEigen.h"
#include "Utils/FileMap.h"
#include "Utils/FileModelParameters.h"

/**
 * Number of sampling per parameters evaluation
 * for log likelyhood maximisation
 */
static const unsigned int samplingNumber = 1000;

/**
 * Learning/Testing data ratio between 0 and 1
 */
static const double learningDataRatio = 0.75;

/**
 * CMA-ES optimization configuration
 */
static const int cmaesElitismLevel = 0;
static const unsigned int cmaesMaxIterations = 200;
static const unsigned int cmaesRestarts = 1;
static const unsigned int cmaesLambda = 10;
static const double cmaesSigma = -1.0;

/**
 * Enable (for testing purpose) 
 * the lens distortion model
 */
static const bool useCameraDistortion = false;

/**
 * Global default geometry data
 */
static Eigen::MatrixXd defaultGeometryData;
static std::map<std::string, size_t> defaultGeometryName;

/**
 * Build and return the geometryData matrix
 * from default data and given parameters vector
 */
Eigen::MatrixXd getGeometryDataFromParameters(
    const Eigen::VectorXd& params)
{
    //Retrieve camera angular offset
    //3: camera angle offset roll (radian)
    double camOffsetRoll = params(3);
    //4: camera angle offset pitch (radian)
    double camOffsetPitch = params(4);
    //5: camera angle offset yaw (radian)
    double camOffsetYaw = params(5);
    //9: neck angle offset roll (radian)
    double neckOffsetRoll = params(9);
    //10: neck angle offset pitch (radian)
    double neckOffsetPitch = params(10);
    //11: neck angle offset yaw (radian)
    double neckOffsetYaw = params(11);
    
    //Copy default geometry model
    Eigen::MatrixXd geometryData = defaultGeometryData;
    //Assign model angular offset 
    //on camera transformation
    geometryData(defaultGeometryName.at("camera"), 0) += camOffsetRoll;
    geometryData(defaultGeometryName.at("camera"), 1) += camOffsetPitch;
    geometryData(defaultGeometryName.at("camera"), 2) += camOffsetYaw;
    geometryData(defaultGeometryName.at("head_yaw"), 0) += neckOffsetRoll;
    geometryData(defaultGeometryName.at("head_yaw"), 1) += neckOffsetPitch;
    geometryData(defaultGeometryName.at("head_yaw"), 2) += neckOffsetYaw;

    return geometryData;
}

/**
 * Build and return the initial 
 * parameters vector
 */
Eigen::VectorXd buildInitialParameters()
{
    //Parameters initialization
    Eigen::VectorXd initParams(useCameraDistortion ? 14 : 12);
    //0: noise aiming pixel (pixel space)
    initParams(0) = 0.05;
    //1: camera aperture width (radian)
    initParams(1) = 80*M_PI/180.0;
    //2: camera aperture height (radian)
    initParams(2) = 50*M_PI/180.0;
    //3: camera angle offset roll (radian)
    initParams(3) = 0.0;
    //4: camera angle offset pitch (radian)
    initParams(4) = 0.0;
    //5: camera angle offset yaw (radian)
    initParams(5) = 0.0;
    //6: imu angle offset roll (radian)
    initParams(6) = 0.0;
    //7: imu angle offset pitch (radian)
    initParams(7) = 0.0;
    //8: imu angle offset yaw (radian)
    initParams(8) = 0.0;
    //9: neck angle offset roll (radian)
    initParams(9) = 0.0;
    //10: neck angle offset pitch (radian)
    initParams(10) = 0.0;
    //11: neck angle offset yaw (radian)
    initParams(11) = 0.0;
    if (useCameraDistortion) {
        //12: distorsion coef 2
        initParams(12) = 0.0;
        //13: distorsion coef 4
        initParams(13) = 0.0;
    }

    return initParams;
}

/**
 * Build and return the 
 * normalization coefficients
 */
Eigen::VectorXd buildNormalizationCoef()
{
    //Parameters initialization
    Eigen::VectorXd normCoef(useCameraDistortion ? 14 : 12);
    //0: noise aiming pixel (pixel space)
    normCoef(0) = 0.05;
    //1: camera aperture width (radian)
    normCoef(1) = 80.0*M_PI/180.0;
    //2: camera aperture height (radian)
    normCoef(2) = 50.0*M_PI/180.0;
    //3: camera angle offset roll (radian)
    normCoef(3) = 2.0*M_PI/180.0;
    //4: camera angle offset pitch (radian)
    normCoef(4) = 2.0*M_PI/180.0;
    //5: camera angle offset yaw (radian)
    normCoef(5) = 2.0*M_PI/180.0;
    //6: imu angle offset roll (radian)
    normCoef(6) = 1.0*M_PI/180.0;
    //7: imu angle offset pitch (radian)
    normCoef(7) = 1.0*M_PI/180.0;
    //8: imu angle offset yaw (radian)
    normCoef(8) = 1.0*M_PI/180.0;
    //9: neck angle offset roll (radian)
    normCoef(9) = 1.0*M_PI/180.0;
    //10: neck angle offset pitch (radian)
    normCoef(10) = 1.0*M_PI/180.0;
    //11: neck angle offset yaw (radian)
    normCoef(11) = 1.0*M_PI/180.0;
    if (useCameraDistortion) {
        //12: distorsion coef 2
        normCoef(12) = 0.01;
        //13: distorsion coef 4
        normCoef(13) = 0.01;
    }

    return normCoef;
}

/**
 * Check and bound parameters to
 * acceptable range.
 * Return non zero cost if
 * parameters are unbounded.
 */
double boundParameters(
    const Eigen::VectorXd& params)
{
    double cost = 0.0;
    //0: noise aiming pixel (pixel space)
    if (params(0) <= 0.0) {
        cost += 1000.0 - 1000.0*(params(0));
    }
    //1: camera aperture width (radian)
    if (params(1) <= 0.0) {
        cost += 1000.0 - 1000.0*(params(1));
    }
    if (params(1) >= M_PI) {
        cost += 1000.0 + 1000.0*(params(1)-M_PI);
    }
    //2: camera aperture height (radian)
    if (params(2) <= 0.0) {
        cost += 1000.0 - 1000.0*(params(2));
    }
    if (params(2) >= M_PI) {
        cost += 1000.0 + 1000.0*(params(2)-M_PI);
    }
    //3: camera angle offset roll (radian)
    if (fabs(params(3)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(3)) - 20.0*M_PI/180.0);
    }
    //4: camera angle offset pitch (radian)
    if (fabs(params(4)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(4)) - 20.0*M_PI/180.0);
    }
    //5: camera angle offset yaw (radian)
    if (fabs(params(5)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(5)) - 20.0*M_PI/180.0);
    }
    //6: imu angle offset roll (radian)
    if (fabs(params(6)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(6)) - 20.0*M_PI/180.0);
    }
    //7: imu angle offset pitch (radian)
    if (fabs(params(7)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(7)) - 20.0*M_PI/180.0);
    }
    //8: imu angle offset yaw (radian)
    if (fabs(params(8)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(8)) - 20.0*M_PI/180.0);
    }
    //9: neck angle offset roll (radian)
    if (fabs(params(9)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(9)) - 20.0*M_PI/180.0);
    }
    //10: neck angle offset pitch (radian)
    if (fabs(params(10)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(10)) - 20.0*M_PI/180.0);
    }
    //11: neck angle offset yaw (radian)
    if (fabs(params(11)) > 20.0*M_PI/180.0) {
        cost += 1000.0 + 1000.0*(fabs(params(11)) - 20.0*M_PI/180.0);
    }
    if (useCameraDistortion) {
        //12: distorsion coef 2
        //13: distorsion coef 4 
    }

    return cost;
}

/**
 * Model initialization function
 */
Leph::HumanoidFixedModel initModel(
    const Eigen::VectorXd& params)
{
    //Build the geometry from parameters
    Eigen::MatrixXd geometryData = 
        getGeometryDataFromParameters(params);

    //Load the model with 
    //the geometry updated
    Leph::HumanoidFixedModel model(
        Leph::SigmabanModel,
        Eigen::MatrixXd(), {},
        geometryData, defaultGeometryName);

    return model;
}

/**
 * Compute and return the estimation
 * (try to predict observation) from
 * given data and using given parameters
 */
Eigen::VectorXd evaluateParameters(
    const Eigen::VectorXd& params,
    const Leph::VectorLabel& data,
    bool noRandom,
    Leph::HumanoidFixedModel& model,
    std::default_random_engine& engine)
{
    //Parse parameters
    //0: noise aiming pixel (pixel space)
    double noiseAimingPixel = params(0);
    //1: camera aperture width (radian)
    Leph::CameraParameters camParams;
    camParams.widthAperture = params(1);
    //2: camera aperture height (radian)
    camParams.heightAperture = params(2);
    //6: imu angle offset roll (radian)
    double imuOffsetRoll = params(6);
    //7: imu angle offset pitch (radian)
    double imuOffsetPitch = params(7);
    //8: imu angle offset yaw (radian)
    double imuOffsetYaw = params(8);

    //Disable caching optimization
    model.get().setAutoUpdate(true);
    //Assign DOF state
    model.setSupportFoot(Leph::HumanoidFixedModel::LeftSupportFoot);
    for (const std::string& name : Leph::NamesDOF) {
        model.get().setDOF(name, data(name));
    }
    //Assign trunk orientation from IMU
    double imuPitch = data("imu_pitch");
    double imuRoll = data("imu_roll");
    Eigen::Matrix3d imuMatrix = 
        Eigen::AngleAxisd(imuOffsetYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
        * Eigen::AngleAxisd(imuPitch + imuOffsetPitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
        * Eigen::AngleAxisd(imuRoll + imuOffsetRoll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    model.setOrientation(imuMatrix);
    //Assign the left foot to origin
    model.get().setDOF("base_x", 0.0);
    model.get().setDOF("base_y", 0.0);
    model.get().setDOF("base_z", 0.0);
    model.get().setDOF("base_yaw", 0.0);
    //Enable model caching
    model.get().setAutoUpdate(false);
    model.get().updateDOFPosition();

    //Retrieve camera aiming point 
    //in pixel space
    Eigen::Vector2d pixelPoint;
    pixelPoint.x() = data("pixel_x");
    pixelPoint.y() = data("pixel_y");

    //Distortion correction model
    if (useCameraDistortion) {
        //15: distorsion coef 2
        double distortionCoef2 = params(12);
        //16: distorsion coef 4
        double distortionCoef4 = params(13);
        double radius2 = pixelPoint.squaredNorm();
        pixelPoint.x() = pixelPoint.x()*(1.0 
            + distortionCoef2*radius2 + distortionCoef4*pow(radius2, 2));
        pixelPoint.y() = pixelPoint.y()*(1.0 
            + distortionCoef2*radius2 + distortionCoef4*pow(radius2, 2));
    }

    //Add random gaussian on recorded pixel
    if (!noRandom) {
        std::normal_distribution<double> distPixel(0.0, noiseAimingPixel);
        pixelPoint.x() += distPixel(engine);
        pixelPoint.y() += distPixel(engine);
    }

    //Compute head view vector in world
    Eigen::Vector3d viewVectorInWorld = 
        model.get().cameraPixelToViewVector(
            camParams, pixelPoint);
    //Compute the cartesian position estimation.
    //(the computed point is in world frame which 
    //is coincident with the left foot).
    Eigen::Vector3d groundEstimation;
    bool isSuccess = model.get().cameraViewVectorToWorld(
        viewVectorInWorld, groundEstimation);
    
    //Return cartesian estimation
    Eigen::VectorXd estimate(2);
    if (!isSuccess) {
        estimate << 1000.0, 1000.0;
    } else {
        estimate << groundEstimation.x(), groundEstimation.y();
    }
    return estimate;
}

/**
 * Use log likellihood maximisation
 * and CMA-ES to find camera model parameters
 * matching given observations
 */
int main(int argc, char** argv)
{
    //Parse user inputs
    if (argc != 3 && argc != 5) {
        std::cout 
            << "Usage: ./app "
            << "calibrationLogs.matrixlabel"
            << " outputPrefix"
            << " [MODEL model.modelparams]" 
            << std::endl;
        std::cout 
            << "Usage: ./app "
            << "calibrationLogs.matrixlabel"
            << " outputPrefix"
            << " [SEED  seed.camparams]" 
            << std::endl;
        return 1;
    }
    std::string logPath = argv[1];
    std::string outPath = argv[2];
    std::string modelPath = "";
    std::string seedPath = "";
    if (argc == 5 && std::string(argv[3]) == "MODEL") {
        modelPath = argv[4];
    }
    if (argc == 5 && std::string(argv[3]) == "SEED") {
        seedPath = argv[4];
    }
    
    //Loading data
    Leph::MatrixLabel logs;
    logs.load(logPath);
    std::cout << "Loading data from " << logPath 
        << ": " << logs.size() << " points" << std::endl;
    
    //Load default model parameters
    Eigen::MatrixXd jointData;
    std::map<std::string, size_t> jointName;
    Eigen::MatrixXd inertiaData;
    std::map<std::string, size_t> inertiaName;
    if (modelPath == "") {
        std::cout << "Loading default Sigmaban model parameters" << std::endl;
        //Load default geometry and inertia
        //data from Sigmaban model
        Leph::HumanoidModel tmpModel(
            Leph::SigmabanModel, "left_foot_tip", false);
        defaultGeometryData = tmpModel.getGeometryData();
        defaultGeometryName = tmpModel.getGeometryName();
        inertiaData = tmpModel.getInertiaData();
        inertiaName = tmpModel.getInertiaName();
        //Build default joint parameters matrix
        Leph::JointModel tmpJoint;
        Eigen::VectorXd jointParams = tmpJoint.getParameters();
        jointData = Eigen::MatrixXd(
            Leph::NamesDOF.size(), jointParams.size());
        size_t tmpIndex = 0;
        for (const std::string& name : Leph::NamesDOF) {
            jointName[name] = tmpIndex;
            jointData.block(
                tmpIndex, 0, 1, jointParams.size()) = 
                jointParams.transpose();
            tmpIndex++;
        }
    } else {
        std::cout << "Loading model parameters from: " 
            << modelPath << std::endl;
        Leph::ReadModelParameters(
            modelPath,
            jointData, jointName,
            inertiaData, inertiaName,
            defaultGeometryData, defaultGeometryName);
    }

    //Get the initial parameters
    Eigen::VectorXd initParams = buildInitialParameters();
    //If given, seed initial parameters
    if (seedPath != "") {
        std::cout << "Loading seed parameters from: " 
            << seedPath << std::endl;
        //Open file
        std::ifstream file(seedPath);
        if (!file.is_open()) {
            throw std::runtime_error(
                "Unable to open file: " + seedPath);
        }
        //Read data from file
        Eigen::VectorXd camData = Leph::ReadEigenVectorFromStream(file);
        Eigen::VectorXd imuData = Leph::ReadEigenVectorFromStream(file);
        Eigen::MatrixXd tmpGeometryData  = 
            Leph::ReadEigenMatrixFromStream(file);
        std::map<std::string, size_t> tmpGeometryName = 
            Leph::ReadMapFromStream<std::string, size_t>(file);
        file.close();
        //Camera aperture width and height
        initParams(1) = camData(0);
        initParams(2) = camData(1);
        //IMU roll, pitch, yaw offset
        initParams(6) = imuData(0);
        initParams(7) = imuData(1);
        initParams(8) = imuData(2);
        //Geometry camera offsets
        initParams(3) = tmpGeometryData(tmpGeometryName.at("camera"), 0);
        initParams(4) = tmpGeometryData(tmpGeometryName.at("camera"), 1);
        initParams(5) = tmpGeometryData(tmpGeometryName.at("camera"), 2);
        //Geometry neck offsets
        initParams(9) = tmpGeometryData(tmpGeometryName.at("head_yaw"), 0);
        initParams(10) = tmpGeometryData(tmpGeometryName.at("head_yaw"), 1);
        initParams(11) = tmpGeometryData(tmpGeometryName.at("head_yaw"), 2);
    }

    //Initialize the logLikelihood 
    //maximisation process
    Leph::LogLikelihoodMaximization
        <Leph::VectorLabel, Leph::HumanoidFixedModel> calibration;
    calibration.setInitialParameters(
        initParams, buildNormalizationCoef());
    calibration.setUserFunctions(evaluateParameters, boundParameters, initModel);
    //Add observations data
    for (size_t i=0;i<logs.size();i++) {
        Eigen::VectorXd obs(2);
        obs << logs[i]("ground_x"), logs[i]("ground_y");
        calibration.addObservation(obs, logs[i]);
    }

    //Start the CMA-ES optimization
    Leph::Plot plot;
    calibration.runOptimization(
        samplingNumber, 
        learningDataRatio,
        cmaesMaxIterations, 
        cmaesRestarts, 
        cmaesLambda, 
        cmaesSigma, 
        cmaesElitismLevel,
        &plot);

    //Display the best found parameters
    Eigen::VectorXd bestParams = calibration.getParameters();
    std::cout << "noisePixel:     " << bestParams(0) << std::endl;
    std::cout << "apertureWidth:  " << bestParams(1)*180.0/M_PI << std::endl;
    std::cout << "apertureHeight: " << bestParams(2)*180.0/M_PI << std::endl;
    std::cout << "CamOffsetRoll:  " << bestParams(3)*180.0/M_PI << std::endl;
    std::cout << "CamOffsetPitch: " << bestParams(4)*180.0/M_PI << std::endl;
    std::cout << "CamOffsetYaw:   " << bestParams(5)*180.0/M_PI << std::endl;
    std::cout << "IMUOffsetRoll:  " << bestParams(6)*180.0/M_PI << std::endl;
    std::cout << "IMUOffsetPitch: " << bestParams(7)*180.0/M_PI << std::endl;
    std::cout << "IMUOffsetYaw:   " << bestParams(8)*180.0/M_PI << std::endl;
    std::cout << "NeckOffsetRoll:  " << bestParams(9)*180.0/M_PI << std::endl;
    std::cout << "NeckOffsetPitch: " << bestParams(10)*180.0/M_PI << std::endl;
    std::cout << "NeckOffsetYaw:   " << bestParams(11)*180.0/M_PI << std::endl;
    if (useCameraDistortion) {
        std::cout << "DistortionCoef2: " << bestParams(12) << std::endl;
        std::cout << "DistortionCoef4: " << bestParams(13) << std::endl;
    }

    //Export the optimized model parameters
    //Imu roll, pitch, yaw in radian
    Eigen::VectorXd imuOffsets(3);
    imuOffsets << bestParams(6), bestParams(7), bestParams(8);
    //Camera angular aperture width, height in radian
    Eigen::VectorXd camApertures(2);
    camApertures << bestParams(1), bestParams(2);
    //Write to file
    std::string camParamsPath = outPath + "cameraModel.params";
    std::cout << "Writing geometry and camera data to: " 
        << camParamsPath << std::endl;
    std::ofstream file(camParamsPath);
    if (!file.is_open()) {
        throw std::runtime_error(
            "Unable to open file: " + camParamsPath);
    }
    Leph::WriteEigenVectorToStream(file, camApertures);
    Leph::WriteEigenVectorToStream(file, imuOffsets);
    Leph::WriteEigenMatrixToStream(file, 
        getGeometryDataFromParameters(bestParams));
    Leph::WriteMapToStream(file, defaultGeometryName);
    file.close();

    //Plot the convergence graphic
    plot.plot("iteration", "all").render();
    
    return 0;
}

