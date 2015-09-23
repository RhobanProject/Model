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
                  joint, ComponentLibrary::getBody("Virtual"), childFrame);
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
    Eigen::Vector3d shoulderToElbow(0.0036, -0.000836, -0.1405);
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
    Eigen::Vector3d elbowMotorOffset(0, 0, 0);
    Eigen::Vector3d headYawMotorOffset(0, 0, -0.04625);
    Eigen::Vector3d headPitchMotorOffset(0, 0, 0);
    // Foot and Hand tips
    Eigen::Vector3d ankleToArchPlate(0.0216, 0.015, -0.0667);
    Eigen::Vector3d toeToToePlate(0.021, -0.00025, -0.0417);
    Eigen::Vector3d elbowToHand(-0.006319, 0, -0.183088);

    // Misc
    double kneeMotorAngle = 18.69 * M_PI / 180;

    RBDL::Model rbdlModel;
    RBDL::Joint jointRoot(RBDL::JointTypeFixed);
    if ( floatingBase ) { jointRoot = ComponentLibrary::floatingBase;}
    rbdlModel.AddBody(rbdlModel.GetBodyId("ROOT"), RBDLMath::SpatialTransform(), jointRoot,
                      ComponentLibrary::getBody("Virtual"), "trunk");
    // torso TODO place accurately
    addFixedBody(rbdlModel,"trunk", "Torso",
                   trunkToHead / 2,
                   "torso");

    // FitPC TODO: place properly
    addFixedBody(rbdlModel,"trunk", "FitPC",
                 trunkToHead / 2 + Eigen::Vector3d(-0.05,0,0),
                 "FitPC");

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
    for (const std::string & side : {"left", "right"}) {
      auto coeff = sideCoeff.at(side);
      auto rot = sideRot.at(side);
      // DOF
      addVirtualDOF(rbdlModel, "trunk", jointYaw,
                    coeff.cwiseProduct(trunkToHip), side + "_hip_yaw");
      addVirtualDOF(rbdlModel, side + "_hip_yaw", jointRoll, side + "_hip_roll");
      addVirtualDOF(rbdlModel, side + "_hip_roll", jointPitch, side + "_hip_pitch");
      addVirtualDOF(rbdlModel, side + "_hip_pitch", jointPitch,
                    coeff.cwiseProduct(hipToKnee), side + "_knee");
      addVirtualDOF(rbdlModel, side + "_knee", jointPitch,
                    coeff.cwiseProduct(kneeToAnkle), side + "_ankle_pitch");
      addVirtualDOF(rbdlModel, side + "_ankle_pitch", jointRoll, side + "_ankle_roll");
      addVirtualDOF(rbdlModel, side + "_ankle_roll", jointPitch,
                    coeff.cwiseProduct(ankleToToe), side + "_toe");
      addVirtualDOF(rbdlModel, "trunk", jointPitch,
                    coeff.cwiseProduct(trunkToShoulder), side + "_shoulder_pitch");
      addVirtualDOF(rbdlModel, side + "_shoulder_pitch", jointRoll, side + "_shoulder_roll");
      addVirtualDOF(rbdlModel, side + "_shoulder_roll", jointPitch,
                    coeff.cwiseProduct(shoulderToElbow), side + "_elbow");
      // Motors
      addFixedBody(rbdlModel,"trunk", "EX106+",
                   SpatialTransform(rotX(M_PI) *rotZ(M_PI/2),
                                    coeff.cwiseProduct(trunkToHip + hipYawMotorOffset)),
                   side + "_hip_yaw_motor");
      addFixedBody(rbdlModel, side + "_hip_roll", "EX106+",
                   SpatialTransform(rotX(-M_PI/2) * rotZ(-M_PI/2),
                                    coeff.cwiseProduct(hipRollMotorOffset)),
                   side + "_hip_roll_motor");
      addFixedBody(rbdlModel, side + "_hip_roll", "EX106+",
                   SpatialTransform(rot * rotY(M_PI) * rotX(- M_PI / 2),
                                    coeff.cwiseProduct(hipPitchMotorOffset)),
                   side + "_hip_pitch_motor");
      addFixedBody(rbdlModel, side + "_hip_pitch", "EX106+",
                   SpatialTransform(rot * rotZ(-kneeMotorAngle) * rotX(M_PI / 2),
                                    coeff.cwiseProduct(hipToKnee + kneeMotorOffset)),
                   side + "_hip_knee_motor");
      addFixedBody(rbdlModel, side + "_ankle_pitch", "EX106+",
                   SpatialTransform(rot * rotX(M_PI/2),
                                    coeff.cwiseProduct(anklePitchMotorOffset)),
                   side + "_ankle_pitch_motor");
      addFixedBody(rbdlModel, side + "_ankle_pitch", "EX106+",
                   SpatialTransform(rotX(M_PI/2) * rotZ(-M_PI/2),
                                    coeff.cwiseProduct(ankleRollMotorOffset)),
                   side + "_ankle_roll_motor");
      addFixedBody(rbdlModel, side + "_ankle_roll", "RX28",
                   SpatialTransform(rot * rotY(M_PI/2) * rotZ(M_PI/2),
                                    coeff.cwiseProduct(ankleToToe)),
                   side + "_toe_motor");
      addFixedBody(rbdlModel, "trunk", "RX64",
                   SpatialTransform(rot * rotY(M_PI) * rotX(- M_PI / 2),
                                    coeff.cwiseProduct(trunkToShoulder + shoulderPitchMotorOffset)),
                   side + "_shoulder_pitch_motor");
      addFixedBody(rbdlModel, side + "_shoulder_roll", "RX64",
                   SpatialTransform(rotX(-M_PI/2) * rotZ(-M_PI/2),
                                    coeff.cwiseProduct(shoulderRollMotorOffset)),
                   side + "_shoulder_roll_motor");
      addFixedBody(rbdlModel, side + "_elbow", "RX64",
                   SpatialTransform(rot * rotZ(M_PI)* rotX(M_PI / 2),
                                    coeff.cwiseProduct(elbowMotorOffset)),
                   side + "_shoulder_elbow_motor");
      // Foot
      addFixedBody(rbdlModel, side + "_ankle_roll", "ArchPlate",
                   coeff.cwiseProduct(ankleToArchPlate),
                   side + "_arch_center");
      addFixedBody(rbdlModel, side + "_toe", "ToePlate",
                   coeff.cwiseProduct(toeToToePlate),
                   side + "_toe_center");

    
      Eigen::Vector3d archSize(0.125,0.092,0);
      Eigen::Vector3d toeSize(0.032, 0.110,0);
      // Virtual Heel
      addFixedBody(rbdlModel, side + "_arch_center", "Virtual",
                   Eigen::Vector3d(-archSize.x()/2, 0, 0),
                   side + "_heel");
      

      std::vector<Eigen::Vector3d> gaugeCoeffs = {Eigen::Vector3d( 0.5, 0.5,0),
                                                  Eigen::Vector3d(-0.5, 0.5,0),
                                                  Eigen::Vector3d(-0.5,-0.5,0),
                                                  Eigen::Vector3d( 0.5,-0.5,0)};
      for (unsigned int i = 0; i < gaugeCoeffs.size(); i++) {
        std::ostringstream archOss, toeOss;
        archOss << side << "_arch_gauge_" << i;
        toeOss << side << "_toe_gauge_" << i;
        addFixedBody(rbdlModel, side + "_arch_center", "Gauge",
                     gaugeCoeffs[i].cwiseProduct(archSize),
                     archOss.str());
        addFixedBody(rbdlModel, side + "_toe_center", "Gauge",
                     gaugeCoeffs[i].cwiseProduct(toeSize),
                     toeOss.str());
      }
      // Hands
      addFixedBody(rbdlModel, side + "_elbow", "Virtual",
                   coeff.cwiseProduct(elbowToHand),
                   side + "_hand");

      // Structure parts
      // TODO: do something more accurate than only dividing by 2 the distance
      addFixedBody(rbdlModel, side + "_hip_pitch", "Femur",
                   coeff.cwiseProduct(hipToKnee / 2),
                   side + "_femur");
      addFixedBody(rbdlModel, side + "_knee", "Tibia",
                   coeff.cwiseProduct(kneeToAnkle / 2),
                   side + "_tibia");
      addFixedBody(rbdlModel, side + "_shoulder_roll", "Arm1",
                   coeff.cwiseProduct(shoulderToElbow / 2),
                   side + "_arm1");
      addFixedBody(rbdlModel, side + "_elbow", "Arm2",
                   coeff.cwiseProduct(elbowToHand / 2),
                   side + "_arm2");
    }
    return rbdlModel;
  }
}
