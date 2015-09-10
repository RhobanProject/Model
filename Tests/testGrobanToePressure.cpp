#include <iostream>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Model/RBDLRootUpdate.h"
#include "Model/Model.hpp"
#include "Model/GrobanToePressureModel.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

namespace RBDL = RigidBodyDynamics;

int main()
{
  //Load model into wrapping class
  Leph::GrobanToePressureModel model("GrosbanToe.urdf","LeftToe");
  std::cout << model.get().getDOF() << std::endl;
    
  //Viewer loop
  Leph::ModelViewer viewer(1200, 900);

  double t = 0;
  double period = 100;
  double amplitude = 0.08;

  
  double baseX =model.get().position("trunk","origin").x();
  double baseY =model.get().position("trunk","origin").y();
  double baseZ =model.get().position("trunk","origin").z() - 0.15;

  Eigen::Vector3d baseRightFootPos = model.get().position("right_toe_tip","origin");

  Eigen::Matrix3d baseTrunkOrientation = model.get().orientation("trunk","origin");
  Eigen::Matrix3d baseRightFootOrientation = model.get().orientation("right_toe_tip","origin");

  Leph::Chrono chrono;


  while (viewer.update()) {

    double diff = sin( t / period);
    // Faking pressures
    model.setPressure("LeftBase",  diff, 0, 0);
    model.setPressure("LeftToe" , -diff, 0, 0);

    model.updateBase();

    double toeAmplitude = 10 * M_PI / 180;
    model.get().setDOF("left_toe", -std::fabs(diff) * toeAmplitude);

    chrono.start("InverseKinematics");

    //Inverse Kinematics
    Leph::InverseKinematics inv(model.get());
    //Declare model degrees of freedom
    inv.addDOF("right_hip_yaw");
    inv.addDOF("right_hip_pitch");
    inv.addDOF("right_hip_roll");
    inv.addDOF("right_knee");
    inv.addDOF("right_ankle_pitch");
    inv.addDOF("right_ankle_roll");
    inv.addDOF("left_hip_yaw");
    inv.addDOF("left_hip_pitch");
    inv.addDOF("left_hip_roll");
    inv.addDOF("left_knee");
    inv.addDOF("left_ankle_pitch");
    inv.addDOF("left_ankle_roll");
    //inv.addDOF("base_x");
    //inv.addDOF("base_y");
    //inv.addDOF("base_z");
    //Declare degree of freedom box bounds 
    //XXX Not fully implemented
    //inv.setLowerBound("left_knee", 0.0);
    //inv.setLowerBound("right_knee", 0.0);


    //Declare target position
    inv.addTargetPosition("flying_foot", "right_toe_tip");
    inv.addTargetPosition("trunk", "trunk");
    inv.addTargetPosition("support_foot", "left_toe_tip");
    //Target orientation
    inv.addTargetOrientation("flying_foot", "right_toe_tip");
    inv.addTargetOrientation("trunk", "trunk");
    inv.addTargetOrientation("support_foot", "left_toe_tip");

    //Update targets
    inv.targetPosition("trunk").x() = baseX+amplitude*sin(2.0*t / period);
    inv.targetPosition("trunk").y() = baseY+amplitude*sin(t / period);
    inv.targetPosition("trunk").z() = baseZ;
    inv.targetPosition("flying_foot") = baseRightFootPos;

    inv.targetOrientation("trunk") = baseTrunkOrientation;
    inv.targetOrientation("flying_foot") = baseRightFootOrientation;


    //Compute Inverse Kinematics
    inv.run(0.0001, 100);
    chrono.stop("InverseKinematics");
    chrono.print();
    std::cout << "ERRORS" << std::endl;
    std::cout << "Flying foot pos: " << inv.errorPosition("flying_foot") << std::endl;
    std::cout << "Support foot pos: " << inv.errorPosition("support_foot") << std::endl;
    std::cout << "Trunk foot pos: " << inv.errorPosition("trunk") << std::endl;
    std::cout << "Flying foot orientation: " << inv.errorOrientation("flying_foot") << std::endl;
    std::cout << "Support foot orientation: " << inv.errorOrientation("support_foot") << std::endl;

    Eigen::Vector3d pt = model.get().position("trunk", "origin");
    viewer.addTrackedPoint(pt); 


    //double angle = (sin(t / period) * amplitude + amplitude) / 2;
    //model.setDOF("left_hip_pitch", - angle);
    //model.setDOF("left_knee", 2 * angle);
    //model.setDOF("left_ankle_pitch", - angle);
    Leph::ModelDraw(model.get(), viewer);
    t += 1;
  }
    
  return 0;
}

