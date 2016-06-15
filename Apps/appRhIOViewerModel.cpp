#include <iostream>
#include <string>
#include <mutex>
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "RhIO.hpp"
#include "RhIOClient.hpp"

int main(int argc, char** argv)
{
    //Parse command line arguments
    if (argc != 3) {
        std::cout 
            << "Usage: ./app robothost [sigmaban|grosban]" 
            << std::endl;
        return -1;
    }
    Leph::RobotType type;
    if (std::string(argv[2]) == "sigmaban") {
        type = Leph::SigmabanModel;
    } else if (std::string(argv[2]) == "grosban") {
        type = Leph::GrosbanModel;
    } else {
        std::cout 
            << "Usage: ./app robothost [sigmaban|grosban]" 
            << std::endl;
        return -1;
    }
    std::string host = std::string(argv[1]);

    //Connection to RhIO server
    std::cout << "Connecting to " << host << " with ";
    std::cout << (type == Leph::SigmabanModel ? "Sigmaban" : "Grosban") 
        << " model" << std::endl;
    RhIO::ClientSub clientSub(
        std::string("tcp://" + host + ":") + RhIO::ServerPubPort);
    RhIO::ClientReq clientReq(
        std::string("tcp://" + host + ":") + RhIO::ServerRepPort);
    
    //Initialize model instances
    Leph::HumanoidFixedPressureModel model(type);
    Leph::HumanoidFixedPressureModel modelGoal(type);
    Leph::ModelViewer viewer(1200, 900);
    viewer.maxTrajectory = 50;
    Leph::Scheduling scheduling;
    scheduling.setFrequency(50.0);
    
    //Mutex protecting model access
    std::mutex mutexModel;
    std::mutex mutexVar;
    
    //Model variables
    double yaw;
    double pitch;
    double roll;
    double pressureLeftWeight;
    double pressureLeftX;
    double pressureLeftY;
    double pressureRightWeight;
    double pressureRightX;
    double pressureRightY;
    std::vector<std::string> servosNames = {
        "head_pitch", "head_yaw", 
        "left_ankle_pitch", "left_ankle_roll", 
        "left_knee",
        "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
        "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
        "right_ankle_pitch", "right_ankle_roll", 
        "right_knee",
        "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
        "right_shoulder_pitch", "right_shoulder_roll", "right_elbow"
    };
    //Setting streaming handler
    clientSub.setHandlerFloat(
        [&mutexVar, &mutexModel, &model, &modelGoal,
        &yaw, &pitch, &roll,
        &pressureLeftWeight, &pressureLeftX, &pressureLeftY,
        &pressureRightWeight, &pressureRightX, &pressureRightY,
        &servosNames]
        (const std::string name, int64_t timestamp, double val) 
    {
        (void)timestamp;
        if (name == "lowlevel/imu/yaw") {
            std::lock_guard<std::mutex> lock(mutexVar);
            yaw = val*M_PI/180.0;
        }
        if (name == "lowlevel/imu/pitch") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pitch = val*M_PI/180.0;
        }
        if (name == "lowlevel/imu/roll") {
            std::lock_guard<std::mutex> lock(mutexVar);
            roll = val*M_PI/180.0;
        }
        for (size_t i=0;i<servosNames.size();i++) {
            if (name == "lowlevel/" + servosNames[i] + "/position") {
                std::lock_guard<std::mutex> lock(mutexModel);
                model.get().setDOF(servosNames[i], val*M_PI/180.0);
            }
            if (name == "lowlevel/" + servosNames[i] + "/goalPosition") {
                std::lock_guard<std::mutex> lock(mutexModel);
                modelGoal.get().setDOF(servosNames[i], val*M_PI/180.0);
            }
        }
        if (name == "lowlevel/left_pressure/weight") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureLeftWeight = val;
        }
        if (name == "lowlevel/left_pressure/x") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureLeftX = val;
        }
        if (name == "lowlevel/left_pressure/y") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureLeftY = val;
        }
        if (name == "lowlevel/right_pressure/weight") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureRightWeight = val;
        }
        if (name == "lowlevel/right_pressure/x") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureRightX = val;
        }
        if (name == "lowlevel/right_pressure/y") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureRightY = val;
        }
    });
    //Enabling value streaming
    clientReq.enableStreamingValue("lowlevel/imu/yaw");
    clientReq.enableStreamingValue("lowlevel/imu/pitch");
    clientReq.enableStreamingValue("lowlevel/imu/roll");
    for (size_t i=0;i<servosNames.size();i++) {
        clientReq.enableStreamingValue(
            "lowlevel/" + servosNames[i] + "/position");
        clientReq.enableStreamingValue(
            "lowlevel/" + servosNames[i] + "/goalPosition");
    }
    clientReq.enableStreamingValue("lowlevel/left_pressure/weight");
    clientReq.enableStreamingValue("lowlevel/left_pressure/x");
    clientReq.enableStreamingValue("lowlevel/left_pressure/y");
    clientReq.enableStreamingValue("lowlevel/right_pressure/weight");
    clientReq.enableStreamingValue("lowlevel/right_pressure/x");
    clientReq.enableStreamingValue("lowlevel/right_pressure/y");
    
    //Main viewer loop
    while (viewer.update()) {
        //Waiting
        scheduling.wait();
        //Model update
        std::lock_guard<std::mutex> lockVar(mutexVar);
        std::lock_guard<std::mutex> lockModel(mutexModel);
        if (pressureLeftWeight+pressureRightWeight > 0.0001) {
            model.setPressure(
                pressureLeftWeight+pressureRightWeight,
                pressureLeftWeight/(pressureLeftWeight+pressureRightWeight),
                pressureRightWeight/(pressureLeftWeight+pressureRightWeight),
                pressureLeftX,
                pressureLeftY,
                pressureRightX,
                pressureRightY);
        }
        model.updateBase();
        modelGoal.updateBase();
        //Rebuilt extrinsic matrix
        Eigen::Matrix3d imuMatrix = 
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        model.setOrientation(
            imuMatrix);
        modelGoal.setOrientation(
            imuMatrix);
        //Model viewer
        Eigen::Vector3d copLeft = model.centerOfPressureLeft("origin");
        Eigen::Vector3d copRight = model.centerOfPressureRight("origin");
        Eigen::Vector3d copMiddle = model.centerOfPressure("origin");
        //Display foot pressure force
        double weightLeft = model.pressureWeight()
            *model.pressureLeftRatio();
        double weightRight = model.pressureWeight()
            *model.pressureRightRatio();
        double boxLength = 0.02;
        viewer.drawBox(0.005, 0.005, boxLength*weightLeft,
            copLeft + Eigen::Vector3d(0, 0, boxLength*weightLeft), 
            Eigen::Matrix3d::Identity(),
            1.0, 0.0, 0.0);
        viewer.drawBox(0.005, 0.005, boxLength*weightRight,
            copRight + Eigen::Vector3d(0, 0, boxLength*weightRight), 
            Eigen::Matrix3d::Identity(),
            0.0, 1.0, 0.0);
        //Display centers of pressures trajectory
        Eigen::Vector3d com = model.get().centerOfMass("origin");
        com.z() = 0.0;
        /*
        viewer.addTrackedPoint(
            copLeft, Leph::ModelViewer::Red);
        viewer.addTrackedPoint(
            copRight, Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            copMiddle, Leph::ModelViewer::Yellow);
        */
        viewer.addTrackedPoint(
            com, Leph::ModelViewer::Red);
        Leph::ModelDraw(modelGoal.get(), viewer);
        Leph::CameraParameters camParams = {74*3.14/180.0, 99*3.14/180.0};
        viewer.drawFrame(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Matrix3d::Identity());
        Leph::ModelDraw(model.get(), viewer);
        Leph::CameraDraw(camParams, model.get(), viewer);
    }
    //Disabling value streaming
    clientReq.disableStreamingValue("lowlevel/imu/yaw");
    clientReq.disableStreamingValue("lowlevel/imu/pitch");
    clientReq.disableStreamingValue("lowlevel/imu/roll");
    for (size_t i=0;i<servosNames.size();i++) {
        clientReq.disableStreamingValue(
            "lowlevel/" + servosNames[i] + "/position");
        clientReq.disableStreamingValue(
            "lowlevel/" + servosNames[i] + "/goalPosition");
    }
    clientReq.disableStreamingValue("lowlevel/left_pressure/weight");
    clientReq.disableStreamingValue("lowlevel/left_pressure/x");
    clientReq.disableStreamingValue("lowlevel/left_pressure/y");
    clientReq.disableStreamingValue("lowlevel/right_pressure/weight");
    clientReq.disableStreamingValue("lowlevel/right_pressure/x");
    clientReq.disableStreamingValue("lowlevel/right_pressure/y");

    return 0;
}

