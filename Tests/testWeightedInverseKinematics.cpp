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
    inv.addDOF("trunk_y");
    inv.addDOF("trunk_z");

    //Declare target position
    inv.addTargetPosition("left_foot" , "left_arch_tip" );
    //inv.addTargetPosition("right_toe" , "left_toe_tip" );
    inv.addTargetCOM();

    Leph::Chrono chrono;
    double t = 0.0;
    while (viewer.update()) {
        t += 0.01;

        //Update impossible targets
        inv.targetPosition("left_foot").x() = 3;
        //inv.targetPosition("right_toe").x() = -3;
        inv.targetCOM().x() = -3;

        inv.weightPosition("left_foot") = Eigen::Vector3d::Constant(1);
        //inv.weightPosition("right_toe") = Eigen::Vector3d::Constant(4);
        inv.weightCOM() = Eigen::Vector3d::Constant(4);

        chrono.start("InverseKinematics");
        //Compute Inverse Kinematics
        inv.run(0.0001, 100);
        chrono.stop("InverseKinematics");
        chrono.print();
        std::cout << "Left foot pos  : " << model.position("left_arch_tip", "origin").x() << std::endl;
        std::cout << "Left foot error: " << inv.errorPosition("left_foot") << std::endl;
        //std::cout << "Right toe pos  : " << model.position("right_toe_tip", "origin").x() << std::endl;
        //std::cout << "Right toe error: " << inv.errorPosition("right_toe") << std::endl;
        std::cout << "COM pos        : " << model.centerOfMass("origin").x() << std::endl;
        std::cout << "COM error      : " << inv.errorCOM() << std::endl;
        
        //Display
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

