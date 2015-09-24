#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/ModelBuilder.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"
#include "Utils/STLibrary.hpp"

#define DEG2RAD(x) (x * M_PI / 180)

int main()
{
    Leph::Model model = Leph::generateGrobanWithToe(true);

    std::cout << model.getDOF() << std::endl;

    Leph::ModelViewer viewer(1200, 900);

    double extraShoulderRoll = 15;
    double feetSpacing = 0.22;
    double leftExtraZ = 0;
    double rightExtraZ = 0;
    double trunkZ = 0.52;
    double xAction = 0;
    double yAction = 0;
    double rollTrunkDeg = 0;

    model.setDOF("left_shoulder_roll" , DEG2RAD( extraShoulderRoll));
    model.setDOF("right_shoulder_roll", DEG2RAD(-extraShoulderRoll));

    Leph::InverseKinematics inv(model);
    //Declare model degrees of freedom
    for (const auto& dof : model.getDOFCategory("lowerBody")) {
      if (dof.find("toe") == std::string::npos) {
        std::cout << "Adding a dof from lowerBody category: " << dof << std::endl;
        inv.addDOF(dof);
      }
    }
    for (const auto& dof : model.getDOFCategory("base")) {
      std::cout << "Adding a dof from base category: " << dof << std::endl;
      inv.addDOF(dof);
    }
    // Declare bounds
    inv.setLowerBound("left_knee", DEG2RAD(15));
    inv.setLowerBound("right_knee", DEG2RAD(15));

    // Declare target Position
    inv.addTargetPosition("LeftFoot", "left_arch_center");
    inv.addTargetPosition("RightFoot", "right_arch_center");
    inv.addTargetPosition("trunk", "trunk");
    inv.addTargetCOM();
    // x,y axis are for COM and z axis for trunk
    inv.weightPosition("trunk") = Eigen::Vector3d(0,0,1);
    inv.weightCOM()             = Eigen::Vector3d(1,1,0);
    //Target orientation
    inv.addTargetOrientation("LeftFoot", "left_arch_center");
    inv.addTargetOrientation("trunk", "trunk");
    inv.addTargetOrientation("RightFoot", "right_arch_center");

    double feetLength = 0.007;//TODO use a rhio version instead
    //Set Positions
    inv.targetPosition("LeftFoot")  = Eigen::Vector3d(feetLength,  feetSpacing/2, leftExtraZ);
    inv.targetPosition("RightFoot") = Eigen::Vector3d(feetLength, -feetSpacing/2, rightExtraZ);
    inv.targetPosition("trunk")     = Eigen::Vector3d(0, 0, trunkZ);
    inv.targetCOM()                 = Eigen::Vector3d(xAction,  yAction, 0);

    //Set Orientations
    inv.targetOrientation("LeftFoot")  = Eigen::Matrix3d::Identity();
    inv.targetOrientation("RightFoot") = Eigen::Matrix3d::Identity();
    inv.targetOrientation("trunk")     = Leph::rotX(DEG2RAD(rollTrunkDeg));

    inv.run(0.0001,50);

    //Declare orientation and dof
    std::vector<std::string> targetDOFs = {"left_hip_roll",
                                           "right_hip_roll",
                                           "left_ankle_roll",
                                           "right_ankle_roll"};
    for (const std::string& dofName : targetDOFs)
    {
      inv.addTargetDOF(dofName, dofName);
    }
    inv.addTargetOrientation("left_foot","left_arch_gauge_0");
    inv.addTargetOrientation("right_foot","right_arch_center");
    //Set incompatible orientaton and dof targets
    for (const std::string& dofName : targetDOFs)
    {
      inv.targetDOF(dofName) = 0;
    }
    inv.targetDOF("left_ankle_roll") = 4 * M_PI / 180;
    inv.targetOrientation("left_foot")  = Eigen::Matrix3d::Identity();
    inv.targetOrientation("right_foot") = Eigen::Matrix3d::Identity();
    //Set Weight of orientation and dof
    inv.weightOrientation("left_foot")  = 1000;
    inv.weightOrientation("right_foot") = 1000;

    Leph::Chrono chrono;
    double t = 0.0;
    while (viewer.update()) {
        t += 0.01;

        std::cout << "weights" << std::endl;
        std::cout << inv.getNamedWeights() << std::endl;

        chrono.start("InverseKinematics");
        //Compute Inverse Kinematics
        //inv.randomDOFNoise();
        inv.run(0.0001, 100);
        chrono.stop("InverseKinematics");
        chrono.print();
        std::cout << "Left foot pos  : " << model.position("left_arch_gauge_0", "origin").x() << std::endl;
        std::cout << "Left foot error: " << inv.errorPosition("left_foot") << std::endl;
        std::cout << "COM pos        : " << model.centerOfMass("origin").x() << std::endl;
        std::cout << "COM error      : " << inv.errorCOM() << std::endl;
        std::cout << "left foot orientation error  : " << inv.errorOrientation("left_foot" ) << std::endl;
        std::cout << "right foot orientation error : " << inv.errorOrientation("right_foot") << std::endl;
        std::cout << "roll value: " << model.getDOF("trunk_roll") * 180 / M_PI << " deg" << std::endl;
        for (const std::string & dofName : targetDOFs) {
          std::cout << dofName << " value: " << model.getDOF(dofName) * 180 / M_PI << " deg" << std::endl;
          std::cout << dofName << " error: " << inv.errorDOF(dofName) * 180 / M_PI << " deg" << std::endl;
        }

        std::cout << model.getDOF() << std::endl;
        
        //Display
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

