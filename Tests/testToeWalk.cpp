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


double cycleLength;
double doubleSupportRatio;//Ratio of time spent in double support
double simpleSupportRatio;//Ratio of time spent in simple support

// Unit is cycle
double simpleSupportTime;
double doubleSupportTime;
double flyingFootTime;

enum SupportType {
  LeftBase,
  LeftBase2LeftToe,
  LeftToe,
  LeftToe2RightBase,
  RightBase,
  RightBase2RightToe,
  RightToe,
  RightToe2LeftBase
};

double getInternalTime(double t)
{
  return fmod(t, cycleLength) / cycleLength;
}

double getPhaseTime(double t)
{
  double internalTime = getInternalTime(t);
  internalTime = fmod(internalTime, 0.25);
  if (internalTime < simpleSupportTime){
    return internalTime / simpleSupportTime;
  }
  return (internalTime - simpleSupportTime) / doubleSupportTime;
}

enum SupportType getSupportType(double t)
{
  double internalTime = getInternalTime(t);
  // Simple Support: Left Base
  if (internalTime < simpleSupportTime)        return SupportType::LeftBase;
  if (internalTime < 0.25)                     return SupportType::LeftBase2LeftToe;
  if (internalTime < 0.25 + simpleSupportTime) return SupportType::LeftToe;
  if (internalTime < 0.5)                      return SupportType::LeftToe2RightBase;
  if (internalTime < 0.5 + simpleSupportTime)  return SupportType::RightToe;
  if (internalTime < 0.75)                     return SupportType::RightToe2RightBase;
  if (internalTime < 0.75 + simpleSupportTime) return SupportType::RightToe;
  if (internalTime < 1)                        return SupportType::RightToe2LeftBase;
  throw std::logic_error("Invalid internal time in getSupportType");
}

int main()
{
  //Load model into wrapping class
  Leph::GrobanToePressureModel model("GrosbanToe.urdf","LeftToe");
  std::cout << model.get().getDOF() << std::endl;
    
  //Viewer loop
  Leph::ModelViewer viewer(1200, 900);

  double t = 0;
  cycleLength = 200;
  doubleSupportRatio = 0.5;//Ratio of time spent in double support
  simpleSupportRatio = 0.5;//Ratio of time spent in simple support

  // Unit is cycle
  simpleSupportTime = simpleSupportRatio / 4;
  doubleSupportTime = doubleSupportRatio / 4;
  flyingFootTime = 2 * simpleSupportTime + doubleSupportTime;

  float feetSpacing = 0.2;
  double trunkZ = 0.45;
  double maxToe = -40 * M_PI / 180;
  double maxFFZ = 0.05;
  double stepLength = 0.3;

  double startLeftX = 0;
  double startRightX = 0;
  double startTrunkX = 0;

  double footBaseToToe = 0.08;//TODO calculate properly
  
  Leph::Chrono chrono;

  std::cerr << "time"
            << ",targetTrunkX"
            << ",targetLeftX"
            << ",targetRightX"
            << ",realTrunkX"
            << ",realLeftX"
            << ",realRightX"
            << std::endl;

  while (viewer.update()) {
    double internalTime = getInternalTime(t);

    // 1. Update pressure TODO improve
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
    enum SupportType support = getSupportPhase(t);
    if (support != getSupportPhase(t-1)) {// Switched phase from last time
      startLeftX  = model.get().position("left_toe_tip" ,"origin").x();
      startRightX = model.get().position("right_toe_tip","origin").x();
      startTrunkX = model.get().position("trunk","origin").x();
    }

    double endLeftX = startLeftX;
    double endRightX = startRightX;
    double endTrunkX = 0;
    double leftZ = 0;
    double rightZ = 0;
    double trunkY = 0;



    double phaseTime = 0; //Internal phase ratio
    // Simple Support: Left Base
    if (internalTime < simpleSupportTime){
      phaseTime = (simpleSupportTime - internalTime) / simpleSupportTime;
      model.get().setDOF("right_toe", phaseTime * maxToe);
      rightZ = phaseTime * maxFFZ;
      endRightX = startLeftX + stepLength * simpleSupportTime / flyingFootTime;
      endTrunkX = startLeftX - footBaseToToe;
      trunkY = model.get().position("left_toe_tip" ,"origin").y();
    }
    // Double Support: Left Base -> Left Toe 
    if (internalTime < 0.25){
      phaseTime = (simpleSupportTime - internalTime) / simpleSupportTime;
      rightZ = maxFFZ;
      endRightX = startLeftX + stepLength * (simpleSupportTime + doubleSupportTime) / flyingFootTime;
      endTrunkX = startLeftX - footBaseToToe;
      trunkY = model.get().position("left_toe_tip" ,"origin").y();
    }
    else if (internalTime < 0.25 + 0.25 * simpleSupportRatio){
      model.get().setDOF("left_toe", (internalTime - 0.25) * 4 * maxToe);
      rightZ = 4 * (0.5 - internalTime) * maxFFZ;
      endRightX = startLeftX + stepLength;
      endTrunkX = startLeftX + stepLength / 2;
      trunkY = model.get().position("left_toe_tip" ,"origin").y();
    }
    else if (internalTime < 0.75){
      model.get().setDOF("left_toe", (0.75 - internalTime) * 4 * maxToe);
      leftZ = 4 * (internalTime - 0.5) * maxFFZ;
      endLeftX = startRightX + stepLength;
      endTrunkX = startRightX + stepLength / 2;
      trunkY = model.get().position("right_toe_tip" ,"origin").y();
    }
    else{
      model.get().setDOF("right_toe", (internalTime - 0.75) * 4 * maxToe);
      leftZ = 4 * (1.0 - internalTime) * maxFFZ;
      endLeftX = startRightX + stepLength;
      endTrunkX = startRightX + stepLength / 2;
      trunkY = model.get().position("right_toe_tip" ,"origin").y();
    }

    double stepTime = fmod(internalTime, 0.5) * 2;
    //if (stepTime == 0) stepTime = 1;
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

    //Set Positions
    inv.targetPosition("LeftFoot")  = Eigen::Vector3d(leftX      ,  feetSpacing/2,      leftZ);
    inv.targetPosition("RightFoot") = Eigen::Vector3d(rightX     , -feetSpacing/2,      rightZ);
    inv.targetPosition("trunk")     = Eigen::Vector3d(trunkX, trunkY, trunkZ);

    //Set Orientations
    inv.targetOrientation("LeftFoot")  = Eigen::Matrix3d::Identity();
    inv.targetOrientation("RightFoot") = Eigen::Matrix3d::Identity();
    inv.targetOrientation("trunk")     = Eigen::Matrix3d::Identity();

    std::cerr << t
              << "," << trunkX
              << "," << leftX
              << "," << rightX
              << "," << model.get().position("trunk"    , "origin").x()
              << "," << model.get().position("left_toe" , "origin").x()
              << "," << model.get().position("right_toe", "origin").x()
              << std::endl;

    chrono.start("InverseKinematics");

    //Compute Inverse Kinematics
    inv.run(0.0001, 100);
    chrono.stop("InverseKinematics");
    std::cout << "trunkXPostInv: " <<
      model.get().position("trunk","origin").x() << std::endl;
    chrono.print();

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

