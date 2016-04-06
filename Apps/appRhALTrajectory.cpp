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
    return Eigen::Vector3d(-0.00557785331559037,  -0.0115849568418458, 0.275);
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

    //Set up RhIO
    RhIO::Root.newCommand("expOver", "Stop the experiment", cmdOver);
    RhIO::Root.newChild("experiment");
    RhIO::Root.newFloat("experiment/trunkPosX")
        ->defaultValue(targetTrunkPos().x())
        ->minimum(-0.1)->maximum(0.1);
    RhIO::Root.newFloat("experiment/trunkPosY")
        ->defaultValue(targetTrunkPos().y())
        ->minimum(-0.1)->maximum(0.1);
    RhIO::Root.newFloat("experiment/trunkAxisX")
        ->defaultValue(targetTrunkAxis().x())
        ->minimum(-1.0)->maximum(1.0);
    RhIO::Root.newFloat("experiment/trunkAxisY")
        ->defaultValue(targetTrunkAxis().y())
        ->minimum(-1.0)->maximum(1.0);
    RhIO::Root.newBool("experiment/exploration")
        ->defaultValue(false);

    //Initialize the model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    Eigen::VectorXd state(2);
    Eigen::VectorXd minBound(2);
    Eigen::VectorXd maxBound(2);
    state << 0.0, 0.0;
    minBound << -0.03, -0.1;
    maxBound << 0.03, 0.1;
    Leph::RandomVelocitySpline exploration(state, minBound, maxBound, 1.0, 4.0);

    //Start cooperative thread
    std::cout << "Starting User thread" << std::endl;
    manager.enableCooperativeThread();

    manager.exitEmergencyState();
    while (!isOver) {
        //Compute model Inverse Kinematics
        Eigen::Vector3d trunkPos = targetTrunkPos();
        Eigen::Vector3d trunkAxis = targetTrunkAxis();
        Eigen::Vector3d footPos = targetFootPos();
        Eigen::Vector3d footAxis = Eigen::Vector3d::Zero();
        trunkPos.x() = RhIO::Root.getFloat("experiment/trunkPosX");
        trunkPos.y() = RhIO::Root.getFloat("experiment/trunkPosY");
        trunkAxis.x() = RhIO::Root.getFloat("experiment/trunkAxisX");
        trunkAxis.y() = RhIO::Root.getFloat("experiment/trunkAxisY");
        //Random exploration
        if (RhIO::Root.getBool("experiment/exploration")) {
            exploration.step(0.02);
            trunkPos.x() += exploration.state()(0);
            trunkAxis.y() += exploration.state()(1);
        }
        //Run Inverse Kinematics
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
    }
    manager.emergencyStop();
    
    //Stop the manager
    std::cout << "Stopping User thread" << std::endl;
    manager.disableCooperativeThread();
    std::cout << "Stopping Manager thread" << std::endl;
    manager.stopManagerThread();

    return 0;
}

