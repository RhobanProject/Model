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
  double amplitude = 0.15;
  float feetSpacing = 0.2;
  double trunkZ = 0.5;

  
  Leph::Chrono chrono;


  while (viewer.update()) {

    double diff = sin( t / period);
    double correctionX = amplitude * sin(t/period);
    double correctionY = amplitude * sin(2*t/period);
    // Faking pressures
    model.setPressure("LeftBase",  diff, 0, 0);
    model.setPressure("LeftToe" , -diff, 0, 0);

    model.updateBase();

    double toeAmplitude = 10 * M_PI / 180;
    //model.get().setDOF("left_toe", -std::fabs(diff) * toeAmplitude);

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
    inv.addDOF("base_x");
    inv.addDOF("base_y");
    inv.addDOF("base_z");
    //inv.addDOF("base_pitch");
    //inv.addDOF("base_yaw");
    //inv.addDOF("base_roll");
    // Declare bounds
    inv.setLowerBound("left_knee", 0.5);
    inv.setLowerBound("right_knee", 0.5);

    // Declare target Position
    inv.addTargetPosition("LeftFoot", "left_arch_tip");
    inv.addTargetPosition("RightFoot", "right_arch_tip");
    inv.addTargetPosition("trunk", "trunk");
    //Target orientation
    inv.addTargetOrientation("LeftFoot", "left_arch_tip");
    inv.addTargetOrientation("trunk", "trunk");
    inv.addTargetOrientation("RightFoot", "right_arch_tip");

    //Set Positions
    inv.targetPosition("LeftFoot")  = Eigen::Vector3d(0,  feetSpacing/2,      0);
    inv.targetPosition("RightFoot") = Eigen::Vector3d(0, -feetSpacing/2,      0);
    inv.targetPosition("trunk")     = Eigen::Vector3d(correctionX,     correctionY, trunkZ);

    //Set Orientations
    inv.targetOrientation("LeftFoot")  = Eigen::Matrix3d::Identity();
    inv.targetOrientation("RightFoot") = Eigen::Matrix3d::Identity();
    inv.targetOrientation("trunk")     = Eigen::Matrix3d::Identity();

    chrono.start("InverseKinematics");

    //Compute Inverse Kinematics
    inv.run(0.0001, 100);
    chrono.stop("InverseKinematics");
    chrono.print();
    std::cout << "ERRORS" << std::endl;
    std::cout << "Left foot pos: " << inv.errorPosition("LeftFoot") << std::endl;
    std::cout << "Right foot pos: " << inv.errorPosition("RightFoot") << std::endl;
    std::cout << "Trunk pos: " << inv.errorPosition("trunk") << std::endl;
    std::cout << "Left foot orientation: " << inv.errorOrientation("LeftFoot") << std::endl;
    std::cout << "Right foot orientation: " << inv.errorOrientation("RightFoot") << std::endl;

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

