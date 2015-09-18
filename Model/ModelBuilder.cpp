#include "ModelBuilder.hpp"

#include "ComponentLibrary.hpp"
#include "Utils/STLibrary.hpp"


namespace Leph {
  using RBDLMath::SpatialTransform;

  void addVirtualDOF(RBDL::Model & model,
                     const std::string & srcFrame,
                     const RBDL::Joint & joint,
                     RBDLMath::SpatialTransform st,
                     const std::string & childFrame)
  {
    model.AddBody(model.GetBodyId(srcFrame.c_str()), st,
                  joint, ComponentLibrary::getBody("virtual"), childFrame);
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
                  RBDL::Joint(RBDL::JointTypeFixed),
                  ComponentLibrary::getBody(bodyName), childFrame);
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

  RBDL::Model generateGrobanWithToe(bool floatingBase)
  {                    
    // DOF dimensions
    Eigen::Vector3d trunkToHip(0, 0.046375, 0);
    Eigen::Vector3d hipToKnee(-0.014956, 0.001, -0.179652);
    Eigen::Vector3d kneeToAnkle(0, 0, -0.25);
    Eigen::Vector3d ankleToToe(0.10132, 0.015, -0.025);
    Eigen::Vector3d trunkToShoulder(-0.0007, 0.103875, 0.247);
    Eigen::Vector3d trunkToHead(0.0072, 0, 0.28025);
    // Motors offsets (should include SpatialTransform due toe rotation around axis)
    // All offsets are given for the left leg
    Eigen::Vector3d hipYawMotorOffset(0, 0, 0.06);
    Eigen::Vector3d hipRollMotorOffset(-0.0511, 0, 0);
    Eigen::Vector3d hipPitchMotorOffset(0, 0, 0);
    Eigen::Vector3d kneeMotorOffset(0, 0, 0);
    Eigen::Vector3d anklePitchMotorOffset(0, 0, 0);
    Eigen::Vector3d ankleRollMotorOffset(-0.0511, 0, 0);
    Eigen::Vector3d shoulderPitchMotorOffset(0, -0.0575, 0);
    Eigen::Vector3d shoulderRollMotorOffset(0, 0, 0);
    Eigen::Vector3d headYawMotorOffset(0, 0, -0.04625);
    Eigen::Vector3d headPitchMotorOffset(0, 0, 0);
    // Foot Tips
    Eigen::Vector3d ankleToArchPlate(0.0216, 0.015, -0.0667);
    Eigen::Vector3d toeToToePlate(0.021, -0.00025, -0.0417);
         
    RBDL::Model rbdlModel;
    RBDL::Joint jointRoot(RBDL::JointTypeFixed);
    if ( floatingBase ) { jointRoot = ComponentLibrary::floatingBase;}
    rbdlModel.AddBody(rbdlModel.GetBodyId("ROOT"), RBDLMath::SpatialTransform(), jointRoot,
                      ComponentLibrary::getBody("virtual"), "trunk");

    // Symetry on XZ plane for directional transform
    std::map<std::string,Eigen::Vector3d> sideCoeff = {{"left" , Eigen::Vector3d(1, 1,1)},
                                                       {"right", Eigen::Vector3d(1,-1,1)}};
    // Pitch motors have not the same orientation for both sides
    std::map<std::string,Eigen::Matrix3d> sideRot = {{"left" , Eigen::Matrix3d::Identity()},
                                                     {"right", rotY(M_PI)}};  // Legs
    const auto & jointRoll  = ComponentLibrary::roll ;
    const auto & jointYaw   = ComponentLibrary::yaw  ;
    const auto & jointPitch = ComponentLibrary::pitch;
    // Head
    addVirtualDOF(rbdlModel, "trunk", jointYaw, trunkToHead, "head_yaw");
    addVirtualDOF(rbdlModel, "head_yaw", jointPitch, "head_pitch");
    addFixedBody(rbdlModel,"trunk", "RX28",
                   SpatialTransform(rotZ(M_PI/2), trunkToHead + headYawMotorOffset),
                   "_head_yaw_motor");
    addFixedBody(rbdlModel,"head_pitch", "RX28",
                   SpatialTransform(rotX(M_PI/2), headPitchMotorOffset),
                   "_head_pitch_motor");
    //TODO fit
    //TODO torso
    //TODO camera
    for (const std::string & legSide : {"left", "right"}) {
      auto coeff = sideCoeff.at(legSide);
      auto rot = sideRot.at(legSide);
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
      addVirtualDOF(rbdlModel, "trunk", jointPitch,
                    coeff.cwiseProduct(trunkToShoulder), legSide + "_shoulder_pitch");
      addVirtualDOF(rbdlModel, legSide + "_shoulder_pitch", jointRoll, legSide + "_shoulder_roll");
      //TODO Elbow+Hand
      // Motors
      addFixedBody(rbdlModel,"trunk", "EX106+",
                   SpatialTransform(rotX(M_PI) *rotZ(M_PI/2),
                                    coeff.cwiseProduct(trunkToHip + hipYawMotorOffset)),
                   legSide + "_hip_yaw_motor");
      addFixedBody(rbdlModel, legSide + "_hip_roll", "EX106+",
                   SpatialTransform(rotX(-M_PI/2) * rotZ(-M_PI/2),
                                    coeff.cwiseProduct(hipRollMotorOffset)),
                   legSide + "_hip_roll_motor");
      addFixedBody(rbdlModel, legSide + "_hip_roll", "EX106+",
                   SpatialTransform(rot * rotY(M_PI) * rotX(- M_PI / 2),
                                    coeff.cwiseProduct(hipPitchMotorOffset)),
                   legSide + "_hip_pitch_motor");
      //TODO update Angle
      addFixedBody(rbdlModel, legSide + "_hip_pitch", "EX106+",
                   SpatialTransform(rot * rotZ(-15 * M_PI / 180) * rotX(M_PI / 2),
                                    coeff.cwiseProduct(hipToKnee + kneeMotorOffset)),
                   legSide + "_hip_knee_motor");
      addFixedBody(rbdlModel, legSide + "_ankle_pitch", "EX106+",
                   SpatialTransform(rot * rotX(M_PI/2),
                                    coeff.cwiseProduct(anklePitchMotorOffset)),
                   legSide + "_ankle_pitch_motor");
      addFixedBody(rbdlModel, legSide + "_ankle_pitch", "EX106+",
                   SpatialTransform(rotX(M_PI/2) * rotZ(-M_PI/2),
                                    coeff.cwiseProduct(ankleRollMotorOffset)),
                   legSide + "_ankle_roll_motor");
      addFixedBody(rbdlModel, legSide + "_ankle_roll", "RX28",
                   SpatialTransform(rot * rotY(M_PI/2) * rotZ(M_PI/2),
                                    coeff.cwiseProduct(ankleToToe)),
                   legSide + "_toe_motor");
      addFixedBody(rbdlModel, "trunk", "RX64",
                   SpatialTransform(rot * rotY(M_PI) * rotX(- M_PI / 2),
                                    coeff.cwiseProduct(trunkToShoulder + shoulderPitchMotorOffset)),
                   legSide + "_shoulder_pitch_motor");
      addFixedBody(rbdlModel, legSide + "_shoulder_roll", "RX64",
                   SpatialTransform(rotX(-M_PI/2) * rotZ(-M_PI/2),
                                    coeff.cwiseProduct(shoulderRollMotorOffset)),
                   legSide + "_shoulder_roll_motor");
      // Foot
      addFixedBody(rbdlModel, legSide + "_ankle_roll", "ArchPlate",
                   coeff.cwiseProduct(ankleToArchPlate),
                   legSide + "_arch_tip");
      addFixedBody(rbdlModel, legSide + "_toe", "ToePlate",
                   coeff.cwiseProduct(toeToToePlate),
                   legSide + "_toe_tip");

    
      Eigen::Vector3d archSize(0.125,0.092,0);
      Eigen::Vector3d toeSize(0.032, 0.110,0);
      std::vector<Eigen::Vector3d> gaugeCoeffs = {Eigen::Vector3d( 0.5, 0.5,0),
                                                  Eigen::Vector3d(-0.5, 0.5,0),
                                                  Eigen::Vector3d(-0.5,-0.5,0),
                                                  Eigen::Vector3d( 0.5,-0.5,0)};
      for (unsigned int i = 0; i < gaugeCoeffs.size(); i++) {
        std::ostringstream archOss, toeOss;
        archOss << legSide << "_arch_gauge_" << i;
        toeOss << legSide << "_toe_gauge_" << i;
        addFixedBody(rbdlModel, legSide + "_arch_tip", "gauge",
                     gaugeCoeffs[i].cwiseProduct(archSize),
                     archOss.str());
        addFixedBody(rbdlModel, legSide + "_toe_tip", "gauge",
                     gaugeCoeffs[i].cwiseProduct(toeSize),
                     toeOss.str());
      }
      // Hands
      //TODO
    }
    return rbdlModel;
  }
}
