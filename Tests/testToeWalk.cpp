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
  double cycleLength = 200;
  float feetSpacing = 0.2;
  double trunkZ = 0.45;
  double maxToe = -40 * M_PI / 180;
  double maxFFZ = 0.05;
  double stepLength = 0.3;

  double startLeftX = 0;
  double startRightX = 0;
  double startTrunkX = 0;

  
  Leph::Chrono chrono;
  std::string previousBase = "";
  std::string newBase = model.getBasename();

  while (viewer.update()) {
    double internalTime = fmod(t, cycleLength) / cycleLength;

    // 1. Update pressure
    model.setPressure("LeftBase" , 0, 0, 0);
    model.setPressure("LeftToe"  , 0, 0, 0);
    model.setPressure("RightBase", 0, 0, 0);
    model.setPressure("RightToe" , 0, 0, 0);
    if (internalTime < 0.25){
      model.setPressure("LeftBase", 1, 0, 0);
    }
    else if (internalTime < 0.5){
      model.setPressure("LeftToe", 1, 0, 0);
    }
    else if (internalTime < 0.75){
      model.setPressure("RightBase", 1, 0, 0);
    }
    else{
      model.setPressure("RightToe", 1, 0, 0);
    }

    // 2. Update base and update start position if necessary
    model.updateBase();
    newBase = model.getBasename();
    if (newBase[0] != previousBase[0]) {// Switch from left to right or right to left
      previousBase = newBase;
      startLeftX  = model.get().position("left_toe" ,"origin").x();
      startRightX = model.get().position("right_toe","origin").x();
      startTrunkX = model.get().position("trunk","origin").x();
    }

    double endLeftX = startLeftX;
    double endRightX = startRightX;
    double endTrunkX = 0;
    double leftZ = 0;
    double rightZ = 0;

    if (internalTime < 0.25){
      model.get().setDOF("right_toe", (0.25 - internalTime) * 4 * maxToe);
      rightZ = 4 * internalTime * maxFFZ;
      endRightX = startLeftX + stepLength;
      endTrunkX = startLeftX + stepLength / 2;
    }
    else if (internalTime < 0.5){
      model.get().setDOF("left_toe", (internalTime - 0.25) * 4 * maxToe);
      rightZ = 4 * (0.5 - internalTime) * maxFFZ;
      endRightX = startLeftX + stepLength;
      endTrunkX = startLeftX + stepLength / 2;
    }
    else if (internalTime < 0.75){
      model.get().setDOF("left_toe", (0.75 - internalTime) * 4 * maxToe);
      leftZ = 4 * (internalTime - 0.5) * maxFFZ;
      endLeftX = startRightX + stepLength;
      endTrunkX = startRightX + stepLength / 2;
    }
    else{
      model.get().setDOF("right_toe", (internalTime - 0.75) * 4 * maxToe);
      leftZ = 4 * (1.0 - internalTime) * maxFFZ;
      endLeftX = startRightX + stepLength;
      endTrunkX = startRightX + stepLength / 2;
    }

    double stepTime = fmod(internalTime, 0.5) * 2;
    if (stepTime == 0) stepTime = 1;
    double leftX  = stepTime * endLeftX  + (1 - stepTime) * startLeftX;
    double rightX = stepTime * endRightX + (1 - stepTime) * startRightX;
    double trunkX = stepTime * endTrunkX + (1 - stepTime) * startTrunkX;


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
    //inv.addDOF("base_pitch");
    //inv.addDOF("base_yaw");
    //inv.addDOF("base_roll");
    // Declare bounds
    inv.setLowerBound("left_knee", 0.5);
    inv.setLowerBound("right_knee", 0.5);

    // Declare target Position
    inv.addTargetPosition("LeftFoot", "left_toe_tip");
    inv.addTargetPosition("RightFoot", "right_toe_tip");
    inv.addTargetPosition("trunk", "trunk");
    //Target orientation
    inv.addTargetOrientation("LeftFoot", "left_toe_tip");
    inv.addTargetOrientation("trunk", "trunk");
    inv.addTargetOrientation("RightFoot", "right_toe_tip");


    double correctionX = 0;
    double correctionY = 0;

    //Set Positions
    inv.targetPosition("LeftFoot")  = Eigen::Vector3d(leftX      ,  feetSpacing/2,      leftZ);
    inv.targetPosition("RightFoot") = Eigen::Vector3d(rightX     , -feetSpacing/2,      rightZ);
    inv.targetPosition("trunk")     = Eigen::Vector3d(trunkX, correctionY, trunkZ);

    //Set Orientations
    inv.targetOrientation("LeftFoot")  = Eigen::Matrix3d::Identity();
    inv.targetOrientation("RightFoot") = Eigen::Matrix3d::Identity();
    inv.targetOrientation("trunk")     = Eigen::Matrix3d::Identity();

    chrono.start("InverseKinematics");

    //Compute Inverse Kinematics
    inv.run(0.0001, 100);
    chrono.stop("InverseKinematics");
    std::cout << "trunkXPostInv: " <<
      model.get().position("trunk","origin").x() << std::endl;
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

