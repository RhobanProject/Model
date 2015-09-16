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
  if (internalTime < 0.5 + simpleSupportTime)  return SupportType::RightBase;
  if (internalTime < 0.75)                     return SupportType::RightBase2RightToe;
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
  cycleLength = 400;
  doubleSupportRatio = 0.5;//Ratio of time spent in double support
  simpleSupportRatio = 1 - doubleSupportRatio;//Ratio of time spent in simple support

  // Unit is cycle
  simpleSupportTime = simpleSupportRatio / 4;
  doubleSupportTime = doubleSupportRatio / 4;
  flyingFootTime = 2 * simpleSupportTime + doubleSupportTime;

  float feetSpacing = 0.2;
  double trunkZ = 0.45;
  double maxToe = -40 * M_PI / 180;
  double maxFFZ = 0.05;
  double stepLength = 0.2;

  double startLeftX = 0;
  double startRightX = 0;
  double startLeftY = 0;
  double startRightY = 0;
  double startTrunkX = 0;
  double startTrunkY = 0;

  double footBaseToToe = 0.08;//TODO calculate properly
  
  Leph::Chrono chrono;

  std::cerr << "time"
            << ",phaseTime"
            << ",targetTrunkX"
            << ",targetTrunkY"
            << ",targetLeftX"
            << ",targetRightX"
            << ",realTrunkX"
            << ",realTrunkY"
            << ",realLeftX"
            << ",realRightX"
            << std::endl;

  while (viewer.update()) {
    double internalTime = getInternalTime(t);

    // 1. Update pressure TODO improve by using continuous data. Maybe add noise
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
    enum SupportType support = getSupportType(t);
    if (support != getSupportType(t-1)) {// Switched phase from last time
      startLeftX  = model.get().position("left_toe_tip" ,"origin").x();
      startLeftY  = model.get().position("left_toe_tip" ,"origin").y();
      startRightX = model.get().position("right_toe_tip","origin").x();
      startRightY = model.get().position("right_toe_tip","origin").y();
      startTrunkX = model.get().position("trunk","origin").x();
      startTrunkY = model.get().position("trunk","origin").y();
    }

    double endLeftX = startLeftX;
    double endLeftY = startLeftY;
    double endRightX = startRightX;
    double endRightY = startRightY;
    double endTrunkX = 0;
    double endTrunkY = 0;
    double leftZ = 0;
    double rightZ = 0;



    double phaseTime = getPhaseTime(t);
    switch(support) {
    case LeftBase: {
      model.get().setDOF("left_toe", 0);
      model.get().setDOF("right_toe", (1 - phaseTime) * maxToe);
      leftZ = 0;
      rightZ = phaseTime * maxFFZ;
      double diffX = (startLeftX + stepLength - startRightX);
      double diffY = (startLeftY - feetSpacing - startRightY);
      endRightX = startRightX + diffX * simpleSupportTime / flyingFootTime;
      endRightY = startRightY + diffY * simpleSupportTime / flyingFootTime;
      endTrunkX = startLeftX - footBaseToToe;
      endTrunkY = startLeftY;
      break;
    }
    case LeftBase2LeftToe: {
      model.get().setDOF("left_toe", 0);
      model.get().setDOF("right_toe", 0);
      leftZ = 0;
      rightZ = maxFFZ;
      double remainingFFTime = flyingFootTime - simpleSupportTime;
      double diffX = (startLeftX + stepLength - startRightX);
      double diffY = (startLeftY - feetSpacing - startRightY);
      endRightX = startRightX + diffX * doubleSupportTime / remainingFFTime;
      endRightY = startRightY + diffY * doubleSupportTime / remainingFFTime;
      endTrunkX = startLeftX;
      endTrunkY = startLeftY;
      break;
    }
    case LeftToe: {
      model.get().setDOF("left_toe", phaseTime * maxToe);
      model.get().setDOF("right_toe", 0);
      leftZ = 0;
      rightZ = (1 - phaseTime) * maxFFZ;
      endRightX = startLeftX + stepLength;
      endRightY = startLeftY - feetSpacing;
      endTrunkX = startLeftX;
      endTrunkY = startLeftY;
      break;
    }
    case LeftToe2RightBase: {
      model.get().setDOF("left_toe", maxToe);
      model.get().setDOF("right_toe", 0);
      leftZ = 0;
      rightZ = 0;
      endTrunkX = startRightX - footBaseToToe;
      endTrunkY = startRightY;
      break;
    }
    case RightBase: {
      model.get().setDOF("right_toe", 0);
      model.get().setDOF("left_toe", (1 - phaseTime) * maxToe);
      rightZ = 0;
      leftZ = phaseTime * maxFFZ;
      endRightX = startRightX;
      double diffX = (startRightX + stepLength - startLeftX);
      double diffY = (startRightY + feetSpacing - startLeftY);
      endLeftX = startLeftX + diffX * simpleSupportTime / flyingFootTime;
      endLeftY = startLeftY + diffY * simpleSupportTime / flyingFootTime;
      endTrunkX = startRightX - footBaseToToe;
      endTrunkY = startRightY;
      break;
    }
    case RightBase2RightToe: {
      model.get().setDOF("right_toe", 0);
      model.get().setDOF("left_toe", 0);
      rightZ = 0;
      leftZ = maxFFZ;
      double remainingFFTime = flyingFootTime - simpleSupportTime;
      double diffX = (startRightX + stepLength - startLeftX);
      double diffY = (startRightY + feetSpacing - startLeftY);
      endLeftX = startLeftX + diffX * doubleSupportTime / remainingFFTime;
      endLeftY = startLeftY + diffY * doubleSupportTime / remainingFFTime;
      endTrunkX = startRightX;
      endTrunkY = startRightY;
      break;
    }
    case RightToe: {
      model.get().setDOF("right_toe", phaseTime * maxToe);
      model.get().setDOF("left_toe" , 0);
      rightZ = 0;
      leftZ = (1 - phaseTime) * maxFFZ;
      endLeftX = startRightX + stepLength;
      endLeftY = startRightY + feetSpacing;
      endTrunkX = startRightX;
      endTrunkY = startRightY;
      break;
    }
    case RightToe2LeftBase: {
      model.get().setDOF("right_toe", maxToe);
      model.get().setDOF("left_toe", 0);
      rightZ = 0;
      leftZ = 0;
      endTrunkX = startLeftX - footBaseToToe;
      endTrunkY = startLeftY;
      break;
    }
    }

    double leftX  = phaseTime * endLeftX  + (1 - phaseTime) * startLeftX;
    double leftY  = phaseTime * endLeftY  + (1 - phaseTime) * startLeftY;
    double rightX = phaseTime * endRightX + (1 - phaseTime) * startRightX;
    double rightY = phaseTime * endRightY + (1 - phaseTime) * startRightY;
    double trunkX = phaseTime * endTrunkX + (1 - phaseTime) * startTrunkX;
    double trunkY = phaseTime * endTrunkY + (1 - phaseTime) * startTrunkY;


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
    inv.setLowerBound("left_knee", DEG2RAD(0.5));
    inv.setLowerBound("right_knee", DEG2RAD(0.5));

    // Declare target Position
    inv.addTargetPosition("LeftFoot", "left_toe_tip");
    inv.addTargetPosition("RightFoot", "right_toe_tip");
    inv.addTargetPosition("trunk", "trunk");
    //Target orientation
    inv.addTargetOrientation("LeftFoot", "left_toe_tip");
    inv.addTargetOrientation("trunk", "trunk");
    inv.addTargetOrientation("RightFoot", "right_toe_tip");

    //Set Positions
    inv.targetPosition("LeftFoot")  = Eigen::Vector3d(leftX , leftY,  leftZ );
    inv.targetPosition("RightFoot") = Eigen::Vector3d(rightX, rightY, rightZ);
    inv.targetPosition("trunk")     = Eigen::Vector3d(trunkX, trunkY, trunkZ);

    //Set Orientations
    inv.targetOrientation("LeftFoot")  = Eigen::Matrix3d::Identity();
    inv.targetOrientation("RightFoot") = Eigen::Matrix3d::Identity();
    inv.targetOrientation("trunk")     = Eigen::Matrix3d::Identity();

    std::cerr << t
              << "," << phaseTime
              << "," << trunkX
              << "," << trunkY
              << "," << leftX
              << "," << rightX
              << "," << model.get().position("trunk"    , "origin").x()
              << "," << model.get().position("trunk"    , "origin").y()
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


    Leph::ModelDraw(model.get(), viewer);
    t += 1;
  }
    
  return 0;
}

