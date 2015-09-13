#include <iostream>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Model/RBDLRootUpdate.h"
#include "Model/Model.hpp"
#include "Model/HumanoidModelWithToe.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

static std::map<std::string,RBDL::Body> bodies;


void addVirtualDOF(RBDL::Model & model,
                   const std::string & srcFrame,
                   const RBDL::Joint & joint,
                   RBDLMath::SpatialTransform st,
                   const std::string & childFrame)
{
  model.AddBody(model.GetBodyId(srcFrame.c_str()), st,
                joint, bodies.at("virtual"), childFrame);
}

// Easier to use when there is no rotation
void addVirtualDOF(RBDL::Model & model,
                   const std::string & srcFrame,
                   const RBDL::Joint & joint,
                   const Eigen::Vector3d & toChild,
                   const std::string & childFrame)
{
  addVirtualDOF(model, srcFrame, joint,
                RBDLMath::SpatialTransform(Eigen::Matrix3d::Identity(), toChild),
                childFrame);
}

// Easier to use when chaining virtual DOFS
void addVirtualDOF(RBDL::Model & model,
                   const std::string & srcFrame,
                   const RBDL::Joint & joint,
                   const std::string & childFrame)
{
  addVirtualDOF(model, srcFrame, joint,
                RBDLMath::SpatialTransform(),
                childFrame);
}   

void addFixedBody(RBDL::Model & model,
                  const std::string & srcFrame,
                  const std::string & bodyName,
                  RBDLMath::SpatialTransform st,
                  const std::string & childFrame)
{
  model.AddBody(model.GetBodyId(srcFrame.c_str()), st,
                RBDL::Joint(RBDL::JointTypeFixed), bodies.at(bodyName), childFrame);
}

// No rotation
void addFixedBody(RBDL::Model & model,
                  const std::string & srcFrame,
                  const std::string & bodyName,
                  const Eigen::Vector3d & offset,
                  const std::string & childFrame)
{
  addFixedBody(model, srcFrame, bodyName,
               RBDLMath::SpatialTransform(Eigen::Matrix3d::Identity(), offset),
               childFrame);
}

// No spatial transformation
void addFixedBody(RBDL::Model & model,
                  const std::string & srcFrame,
                  const std::string & bodyName,
                  const std::string & childFrame)
{
  addFixedBody(model, srcFrame, bodyName,
               RBDLMath::SpatialTransform(),
               childFrame);
}
          

int main()
{
  bodies["virtual"].mMass = 0.001;//RBDL automatically join bodies when using a fixed joint and join cannot be operatoed on two bodies with a 0 mass + Model virtual body name not implemented
  bodies["virtual"].mCenterOfMass = Eigen::Vector3d::Zero();
  bodies["virtual"].mInertia = Eigen::Matrix3d::Identity();
  bodies["virtual"].mIsVirtual = true;
  bodies["EX106+"].mMass = 0.154;
  bodies["EX106+"].mCenterOfMass = Eigen::Vector3d::Zero();
  bodies["EX106+"].mInertia = Eigen::Matrix3d::Identity();
  bodies["EX106+"].mIsVirtual = false;
  bodies["RX64"].mMass = 0.125;
  bodies["RX64"].mCenterOfMass = Eigen::Vector3d::Zero();
  bodies["RX64"].mInertia = Eigen::Matrix3d::Identity();
  bodies["RX64"].mIsVirtual = false;
  bodies["RX28"].mMass = 0.072;
  bodies["RX28"].mCenterOfMass = Eigen::Vector3d::Zero();
  bodies["RX28"].mInertia = Eigen::Matrix3d::Identity();
  bodies["RX28"].mIsVirtual = false;
  bodies["ArchPlate"].mMass = 0.350;//TODO measure
  bodies["ArchPlate"].mCenterOfMass = Eigen::Vector3d::Zero();
  bodies["ArchPlate"].mInertia = Eigen::Matrix3d::Identity();
  bodies["ArchPlate"].mIsVirtual = false;
  bodies["ToePlate"].mMass = 0.150;///TODO measure
  bodies["ToePlate"].mCenterOfMass = Eigen::Vector3d::Zero();
  bodies["ToePlate"].mInertia = Eigen::Matrix3d::Identity();
  bodies["ToePlate"].mIsVirtual = false;
  bodies["pressure"].mMass = 0.002;///TODO measure
  bodies["pressure"].mCenterOfMass = Eigen::Vector3d::Zero();
  bodies["pressure"].mInertia = Eigen::Matrix3d::Identity();
  bodies["pressure"].mIsVirtual = false;

  bool addFloatingBase = true;

  // DOF dimensions
  Eigen::Vector3d trunkToHip(0, 0.046375, 0);
  Eigen::Vector3d hipToKnee(-0.014956, 0, -0.179652);
  Eigen::Vector3d kneeToAnkle(0, 0, -0.25);
  Eigen::Vector3d ankleToToe(0.10132, 0.015, -0.025);
  // Motors offsets (should include SpatialTransform due toe rotation around axis)
  Eigen::Vector3d hipYawMotorOffset(0, 0, 0.05);
  Eigen::Vector3d hipRollMotorOffset(-0.05, 0, 0);
  Eigen::Vector3d hipPitchMotorOffset(0, 0, 0);
  Eigen::Vector3d kneeMotorOffset(0, 0, 0);
  Eigen::Vector3d anklePitchMotorOffset(0, 0, 0);
  Eigen::Vector3d ankleRollMotorOffset(-0.05, 0, 0);
  // Foot Tips
  Eigen::Vector3d ankleToArchPlate(0.0216, 0.015, -0.0667);
  Eigen::Vector3d toeToToePlate(0.021, 0, -0.0417);

  RBDL::Joint jointRoot;
  RBDL::Joint jointFixed(RBDL::JointTypeFixed);
  RBDL::Joint jointRoll (RBDL::JointTypeRevolute, Eigen::Vector3d(1,0,0));
  RBDL::Joint jointPitch(RBDL::JointTypeRevolute, Eigen::Vector3d(0,1,0));
  RBDL::Joint jointYaw  (RBDL::JointTypeRevolute, Eigen::Vector3d(0,0,1));
  if (addFloatingBase) {
    jointRoot = RBDL::Joint(
      RBDLMath::SpatialVector(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
      RBDLMath::SpatialVector(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
      RBDLMath::SpatialVector(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      RBDLMath::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
      RBDLMath::SpatialVector(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
      RBDLMath::SpatialVector(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  } else {
    jointRoot = RBDL::Joint(RBDL::JointTypeFixed);
  }
                             
  RBDL::Model rbdlModel;
  rbdlModel.AddBody(rbdlModel.GetBodyId("ROOT"), RBDLMath::SpatialTransform(), jointRoot,
                    bodies.at("virtual"), "trunk");
  std::map<std::string,Eigen::Vector3d> sideCoeff = {{"left", Eigen::Vector3d(1,1,1)},
                                                     {"right", Eigen::Vector3d(1,-1,1)}};

  // Legs
  for (const std::string & legSide : {"left", "right"}) {
    auto coeff = sideCoeff.at(legSide);
    // DOF
    addVirtualDOF(rbdlModel, "trunk", jointYaw,
                  coeff.cwiseProduct(trunkToHip), legSide + "_hip_yaw");
    addVirtualDOF(rbdlModel, legSide + "_hip_yaw", jointRoll, legSide + "_hip_roll");
    addVirtualDOF(rbdlModel, legSide + "_hip_roll", jointPitch, legSide + "_hip_pitch");
    addVirtualDOF(rbdlModel, legSide + "_hip_pitch", jointPitch,
                  coeff.cwiseProduct(hipToKnee), legSide + "_knee");
    addVirtualDOF(rbdlModel, legSide + "_knee", jointPitch,
                  coeff.cwiseProduct(kneeToAnkle), legSide + "_ankle_pitch");
    addVirtualDOF(rbdlModel, legSide + "_ankle_pitch", jointRoll, legSide + "_ankle_roll");
    addVirtualDOF(rbdlModel, legSide + "_ankle_roll", jointPitch,
                  coeff.cwiseProduct(ankleToToe), legSide + "_toe");
    // Motors
    addFixedBody(rbdlModel,"trunk", "EX106+", coeff.cwiseProduct(trunkToHip + hipYawMotorOffset),
                 legSide + "_hip_yaw_motor");
    addFixedBody(rbdlModel, legSide + "_hip_yaw", "EX106+", coeff.cwiseProduct(hipRollMotorOffset),
                 legSide + "_hip_roll_motor");
    addFixedBody(rbdlModel, legSide + "_hip_roll", "EX106+", coeff.cwiseProduct(hipPitchMotorOffset),
                 legSide + "_hip_pitch_motor");
    addFixedBody(rbdlModel, legSide + "_hip_pitch", "EX106+",
                 coeff.cwiseProduct(hipToKnee + kneeMotorOffset),
                 legSide + "_hip_knee_motor");
    addFixedBody(rbdlModel, legSide + "_knee", "EX106+",
                 coeff.cwiseProduct(kneeToAnkle + anklePitchMotorOffset),
                 legSide + "_ankle_pitch_motor");
    addFixedBody(rbdlModel, legSide + "_ankle_pitch", "EX106+",
                 coeff.cwiseProduct(ankleRollMotorOffset),
                 legSide + "_ankle_roll_motor");
    addFixedBody(rbdlModel, legSide + "_ankle_roll", "RX28",
                 coeff.cwiseProduct(ankleToToe),
                 legSide + "_toe_motor");
    // Foot
    addFixedBody(rbdlModel, legSide + "_ankle_roll", "ArchPlate",
                 coeff.cwiseProduct(ankleToArchPlate),
                 legSide + "_arch_tip");
    addFixedBody(rbdlModel, legSide + "_toe", "ToePlate",
                 coeff.cwiseProduct(toeToToePlate),
                 legSide + "_toe_tip");

    
    Eigen::Vector3d archSize(0.125,0.092,0);
    Eigen::Vector3d toeSize(0.032, 0.110,0);
    std::vector<Eigen::Vector3d> pressureCoeffs = {Eigen::Vector3d( 0.5, 0.5,0),
                                                   Eigen::Vector3d(-0.5, 0.5,0),
                                                   Eigen::Vector3d(-0.5,-0.5,0),
                                                   Eigen::Vector3d( 0.5,-0.5,0)};
    for (unsigned int i = 0; i < pressureCoeffs.size(); i++) {
      std::ostringstream archOss, toeOss;
      archOss << legSide << "_arch_pressure" << i << std::endl;
      toeOss << legSide << "_toe_pressure" << i << std::endl;
      addFixedBody(rbdlModel, legSide + "_arch_tip", "pressure",
                   pressureCoeffs[i].cwiseProduct(archSize),
                   archOss.str());
      addFixedBody(rbdlModel, legSide + "_toe_tip", "pressure",
                   pressureCoeffs[i].cwiseProduct(toeSize),
                   toeOss.str());
    }
    
  }

  Leph::Model model(rbdlModel);
  std::cout << model.getDOF() << std::endl;
    
  //Viewer loop
  Leph::ModelViewer viewer(1200, 900);

  double t = 0;
  double period = 200;
  double amplitude = 30 * M_PI / 180;

  while (viewer.update()) {

    //model.setDOF("left_hip_yaw", amplitude * sin(1 * M_PI * t / period));
    //model.setDOF("left_hip_roll", amplitude * sin(2 * M_PI * t / period));
    //model.setDOF("left_hip_pitch", amplitude * sin(4 * M_PI * t / period));
    Leph::ModelDraw(model, viewer);
    t += 1;
  }
    
  return 0;
}
