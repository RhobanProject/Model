#include <iostream>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFloatingModel.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"

int main()
{
    Leph::HumanoidFloatingModel model("../../Data/sigmaban.urdf");
    model.putOnGround();

    Leph::ModelViewer viewer(1200, 900);

    //Inverse Kinematics
    Leph::InverseKinematics inv(model);
    //Declare model degrees of freedom
    inv.addDOF("right hip pitch");
    inv.addDOF("right hip roll");
    inv.addDOF("right knee");
    inv.addDOF("right foot pitch");
    inv.addDOF("right foot roll");
    inv.addDOF("left hip pitch");
    inv.addDOF("left hip roll");
    inv.addDOF("left knee");
    inv.addDOF("left foot pitch");
    inv.addDOF("left foot roll");
    inv.addDOF("base Tx");
    inv.addDOF("base Ty");
    inv.addDOF("base Tz");
    //Declare degree of freefom box bounds 
    //XXX Not fully implemented
    inv.setLowerBound("left knee", 0.0);
    inv.setLowerBound("right knee", 0.0);

    //Declare target position
    inv.addTargetPosition("flying foot", "right foot tip");
    inv.addTargetPosition("support foot", "left foot tip");
    //target of center of mass
    inv.addTargetCOM();
    inv.targetCOM().z() -= 0.05;
    //Target orientation
    inv.addTargetOrientation("flying foot", "right foot tip");
    inv.addTargetOrientation("support foot", "left foot tip");
    
    Leph::Chrono chrono;
    double t = 0.0;
    while (viewer.update()) {
        t += 0.01;

        //Update targets
        inv.targetPosition("flying foot").z() = 0.05+0.02*sin(t);
        inv.targetPosition("flying foot").x() = 0.05+0.02*sin(2.0*t);
        inv.targetCOM().y() = 0.01*sin(t);
        inv.targetCOM().x() = 0.01*sin(2.0*t);

        chrono.start("InverseKinematics");
        //Compute Inverse Kinematics
        inv.run(0.0001, 100);
        chrono.stop("InverseKinematics");
        chrono.print();
        std::cout << "ERRORS" << std::endl;
        std::cout << "COM pos: " << inv.errorCOM() << std::endl;
        std::cout << "Flying foot pos: " << inv.errorPosition("flying foot") << std::endl;
        std::cout << "Support foot pos: " << inv.errorPosition("support foot") << std::endl;
        std::cout << "Flying foot orientation: " << inv.errorOrientation("flying foot") << std::endl;
        std::cout << "Support foot orientation: " << inv.errorOrientation("support foot") << std::endl;
        
        //Display
        Eigen::Vector3d pt = model.centerOfMass("origin");
        viewer.addTrackedPoint(pt);    
        Leph::ModelDraw(model, viewer);
    }

    return 0;
}

