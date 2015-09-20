#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidModelWithToe.hpp"
#include "Model/ModelBuilder.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"

int main()
{
    Leph::RBDL::Model rbdlModel = Leph::generateGrobanWithToe(true);
    Leph::HumanoidModelWithToe model(rbdlModel);

    std::cout << model.getDOF() << std::endl;

    Leph::ModelViewer viewer(1200, 900);

    //Inverse Kinematics
    Leph::InverseKinematics inv(model);
    //Declare model degrees of freedom
    inv.addDOF("trunk_x");
    //inv.addDOF("trunk_y");
    //inv.addDOF("trunk_z");
    inv.addDOF("trunk_roll");
    inv.addDOF("left_hip_roll");
    inv.addDOF("right_hip_roll");
    inv.addDOF("left_ankle_roll");
    inv.addDOF("right_ankle_roll");

    //Declare target position
    inv.addTargetPosition("left_foot" , "left_arch_gauge_0" );
    inv.addTargetCOM();
    //Set impossible targets
    inv.targetPosition("left_foot").x() = 0;
    inv.targetCOM().x() = -3;
    //Set different weight
    inv.weightPosition("left_foot") = Eigen::Vector3d(1, 0, 0);
    inv.weightCOM() = Eigen::Vector3d(0, 0, 0);

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

        chrono.start("InverseKinematics");
        //Compute Inverse Kinematics
        inv.randomDOFNoise();
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

