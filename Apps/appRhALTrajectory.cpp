#include <iostream>
#include <thread>
#include <chrono>
#include <RhAL.hpp>
#include <RhIO.hpp>
#include "Spline/SplineContainer.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Utils/AxisAngle.h"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/RandomVelocitySpline.hpp"
#include "RhAL/RhALUtils.h"

//Static single support pose
static Eigen::Vector3d targetTrunkPos()
{
    return Eigen::Vector3d(-0.00557785331559037,  -0.0115849568418458, 0.28);
}
static Eigen::Vector3d targetTrunkAxis()
{
    return Eigen::Vector3d(-0.672036398746933, 0.0743358280850477, 0.0028323027017884);
}
static Eigen::Vector3d targetFootPos()
{
    return Eigen::Vector3d(0.0208647084129351, -0.095, 0.0591693358237435);
}

//Global variables
static bool isOver = false;

//RhIO Command
std::string cmdOver(std::vector<std::string> argv)
{
    (void)argv;
    isOver = true;
    return "Over";
}

int main()
{
    //Initialize the Manager
    RhAL::StandardManager manager;
    manager.readConfig("chewbacca_rhal.json");
    std::cout << "Scanning the bus..." << std::endl;
    manager.scan();
    
    //Start the Manager
    std::cout << "Starting Manager Thread" << std::endl;
    manager.startManagerThread();
    std::cout << "Starting RhIO binding" << std::endl;
    RhAL::RhIOBinding binding(manager);

    //Set up RhIO commands
    RhIO::Root.newCommand("expOver", "Stop the experiment", cmdOver);

    //Initialize the model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);

    //Start cooperative thread
    std::cout << "Starting User thread" << std::endl;
    manager.enableCooperativeThread();

    while (!isOver) {
        //Compute model Inverse Kinematics
        Eigen::Vector3d trunkPos = targetTrunkPos();
        Eigen::Vector3d trunkAxis = targetTrunkAxis();
        Eigen::Vector3d footPos = targetFootPos();
        Eigen::Vector3d footAxis = Eigen::Vector3d::Zero();
        bool isSuccess = model.trunkFootIK(
            Leph::HumanoidFixedModel::LeftSupportFoot,
            trunkPos,
            Leph::AxisToMatrix(trunkAxis),
            footPos,
            Leph::AxisToMatrix(footAxis));
        if (!isSuccess) {
            std::cout << "IK ERROR" << std::endl;
            break;
        }
        //Write target to RhAL
        RhALWriteStateGoal(manager, model.get(), true, false, false);
        //Wait next Manager cycle
        manager.waitNextFlush();
        std::cout << "User cycle" << std::endl;
    }
    
    //Stop the manager
    std::cout << "Stopping User thread" << std::endl;
    manager.disableCooperativeThread();
    std::cout << "Stopping Manager thread" << std::endl;
    manager.stopManagerThread();

    return 0;
}

