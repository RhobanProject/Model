#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/ModelBuilder.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"

int main()
{
    Leph::Model model = Leph::generateGrobanWithToe(true);

    std::cout << model.getDOF() << std::endl;

    Leph::ModelViewer viewer(1200, 900);

    //Inverse Kinematics
    Leph::InverseKinematics inv(model);
    //Declare model degrees of freedom
    inv.addDOF("trunk_x");
    inv.addDOF("trunk_y");
    inv.addDOF("trunk_z");
    //inv.addDOF("trunk_roll");
    //inv.addDOF("left_hip_roll");
    //inv.addDOF("right_hip_roll");
    //inv.addDOF("left_ankle_roll");
    //inv.addDOF("right_ankle_roll");

    //Declare target position
    inv.addTargetPosition("left_foot" , "left_arch_center" );
    inv.addTargetCOM();
    //Set impossible targets
    inv.targetPosition("left_foot").x() = 0;
    inv.targetCOM().x() = -5;
    inv.targetPosition("left_foot").y() = 0;
    inv.targetCOM().y() = -5;
    inv.targetPosition("left_foot").z() = 0;
    inv.targetCOM().z() = -5;
    //Set different weight
    inv.weightPosition("left_foot") = Eigen::Vector3d(1, 1, 1);
    inv.weightCOM() = Eigen::Vector3d(4, 4, 4);

    //Declare orientation and dof
    //std::vector<std::string> targetDOFs = {"left_hip_roll",
    //                                       "right_hip_roll",
    //                                       "left_ankle_roll",
    //                                       "right_ankle_roll"};
    //for (const std::string& dofName : targetDOFs)
    //{
    //  inv.addTargetDOF(dofName, dofName);
    //}
    //inv.addTargetOrientation("left_foot","left_arch_gauge_0");
    //inv.addTargetOrientation("right_foot","right_arch_center");
    ////Set incompatible orientaton and dof targets
    //for (const std::string& dofName : targetDOFs)
    //{
    //  inv.targetDOF(dofName) = 0;
    //}
    //inv.targetDOF("left_ankle_roll") = 4 * M_PI / 180;
    //inv.targetOrientation("left_foot")  = Eigen::Matrix3d::Identity();
    //inv.targetOrientation("right_foot") = Eigen::Matrix3d::Identity();
    ////Set Weight of orientation and dof
    //inv.weightOrientation("left_foot")  = 1000;
    //inv.weightOrientation("right_foot") = 1000;

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

        std::cout << "ERRORS" << std::endl << inv.getNamedErrors();
        
        //Display
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

