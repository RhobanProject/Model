#include <iostream>
#include <string>
#include <mutex>
#include "Model/HumanoidFixedPressureModel.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Utils/Scheduling.hpp"
#include "RhIO.hpp"
#include "ClientReq.hpp"
#include "ClientSub.hpp"

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
    Leph::ModelViewer viewer(1200, 900);
    Leph::Scheduling scheduling;
    scheduling.setFrequency(50.0);
    
    //Mutex protecting model access
    std::mutex mutexModel;
    std::mutex mutexVar;
    
    //Model variables
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
    double pressureW;
    double pressureW1;
    double pressureW2;
    double pressureX1;
    double pressureY1;
    double pressureX2;
    double pressureY2;
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
        [&mutexVar, &mutexModel, &model, &pitch, &roll, &yaw,
        &pressureW, &pressureW1, &pressureW2, 
        &pressureX1, &pressureX2, &pressureY1, &pressureY2, 
        &servosNames]
        (const std::string name, int64_t timestamp, double val) 
    {
        (void)timestamp;
        if (name == "sensors/Pitch/value") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pitch = val;
        }
        if (name == "sensors/Roll/value") {
            std::lock_guard<std::mutex> lock(mutexVar);
            roll = val;
        }
        if (name == "sensors/GyroYaw/value") {
            std::lock_guard<std::mutex> lock(mutexVar);
            yaw = val;
        }
        for (size_t i=0;i<servosNames.size();i++) {
            if (name == "servos/" + servosNames[i] + "/angle") {
                std::lock_guard<std::mutex> lock(mutexModel);
                model.get().setDOF(servosNames[i], val*M_PI/180.0);
            }
        }
        if (name == "pressure/weight") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureW = val;
        }
        if (name == "pressure/leftRatio") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureW1 = val;
        }
        if (name == "pressure/rightRatio") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureW2 = val;
        }
        if (name == "pressure/leftX") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureX1 = val;
        }
        if (name == "pressure/leftY") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureY1 = val;
        }
        if (name == "pressure/rightX") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureX2 = val;
        }
        if (name == "pressure/rightY") {
            std::lock_guard<std::mutex> lock(mutexVar);
            pressureY2 = val;
        }
    });
    //Enabling value streaming
    clientReq.enableStreamingValue("sensors/Pitch/value");
    clientReq.enableStreamingValue("sensors/Roll/value");
    clientReq.enableStreamingValue("sensors/GyroYaw/value");
    for (size_t i=0;i<servosNames.size();i++) {
        clientReq.enableStreamingValue(
            "servos/" + servosNames[i] + "/angle");
    }
    clientReq.enableStreamingValue("pressure/weight");
    clientReq.enableStreamingValue("pressure/leftRatio");
    clientReq.enableStreamingValue("pressure/rightRatio");
    clientReq.enableStreamingValue("pressure/leftX");
    clientReq.enableStreamingValue("pressure/leftY");
    clientReq.enableStreamingValue("pressure/rightX");
    clientReq.enableStreamingValue("pressure/rightY");
    
    //Main viewer loop
    while (viewer.update()) {
        //Waiting
        scheduling.wait();
        //Model update
        std::lock_guard<std::mutex> lockVar(mutexVar);
        std::lock_guard<std::mutex> lockModel(mutexModel);
        model.setPressure(
            pressureW/1000.0,
            pressureW1,
            pressureW2,
            pressureX1/100.0,
            pressureY1/100.0,
            pressureX2/100.0,
            pressureY2/100.0);
        model.updateBase();
        model.setOrientation(
            pitch*M_PI/180.0, 
            -roll*M_PI/180.0);
        model.setYaw(model.getSupportFoot(), yaw*M_PI/180.0);
        //Model viewer
        Eigen::Vector3d copLeft = model.centerOfPressureLeft("origin");
        Eigen::Vector3d copRight = model.centerOfPressureRight("origin");
        Eigen::Vector3d copMiddle = model.centerOfPressure("origin");
        //Display foot pressure force
        viewer.drawBox(0.005, 0.005, 0.1*model.pressureLeftRatio(),
            copLeft + Eigen::Vector3d(0, 0, 0.1*model.pressureLeftRatio()), 
            Eigen::Matrix3d::Identity(),
            1.0, 0.0, 0.0);
        viewer.drawBox(0.005, 0.005, 0.1*model.pressureRightRatio(),
            copRight + Eigen::Vector3d(0, 0, 0.1*model.pressureRightRatio()), 
            Eigen::Matrix3d::Identity(),
            0.0, 1.0, 0.0);
        //Display centers of pressures trajectory
        viewer.addTrackedPoint(
            copLeft, Leph::ModelViewer::Red);
        viewer.addTrackedPoint(
            copRight, Leph::ModelViewer::Green);
        viewer.addTrackedPoint(
            copMiddle, Leph::ModelViewer::Yellow);
        Leph::ModelDraw(model.get(), viewer);
    }

    return 0;
}

