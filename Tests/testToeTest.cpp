#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/ModelBuilder.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"
#include "Utils/STLibrary.hpp"

#define DEG2RAD(x) (x * M_PI / 180.0)

int main()
{
    Leph::Model model = Leph::generateGrobanWithToe(true);

    std::cout << model.getDOF() << std::endl;

    Leph::ModelViewer viewer(1200, 900);

    double extraShoulderRoll = 15;
    double feetSpacing = 0.22;
    double leftExtraZ = 0;
    double rightExtraZ = 0;
    double trunkZ = 0.48;
    double xAction = 0;
    double yAction = 0.15;
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

        std::cout << "ERRORS" << std::endl;
        std::cout << inv.getNamedErrors() << std::endl;
        std::cout << "TARGETS" << std::endl;
        std::cout << inv.getNamedTargets() << std::endl;
        std::cout << "DOFSUBSET" << std::endl;
        std::cout << inv.getNamedDOFSubset() << std::endl;
        
        //Display
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

