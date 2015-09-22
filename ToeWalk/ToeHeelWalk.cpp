#include "ToeHeelWalk.hpp"

#include "ModelBuilder.hpp"

using Eigen::Vector3d;

namespace Leph {
  
  ToeHeelWalk::ToeHeelWalk()
    : initialized(false), phaseStart(0),
      phase(Phase::Waiting), lastPhase(Phase::Waiting)
      placingHeelTime(1.0),
      liftingHeelTime(2.0),
      switchWeightTime(2.0),
      liftingFootTime(2.0),
      stepX(0),
      simModel(generateGrobanWithToe()),
      side("none"),
      oppSide("none")
  {
  }

  void ToeHeelWalk::swapSide()
  {
    std::string tmp = side;
    side = oppSide;
    oppSide = tmp;
  }

  void ToeHeelWalk::nextPhase()
  {
    // Set side and phase appopriately
    switch(phase) {
    case Waiting:
      phase = Phase::SwitchWeight;
      side = "right"; oppSide = "left";
      break;
    case SwitchWeight:
      phase = Phase::LiftingArch;
      swapSide();
      break;
    case LiftingArch:
      phase = Phase::FlyingFoot;
      break;
    case FlyingFoot:
      phase = Phase::PlacingHeel;
      break;
    case PlacingHeel:
      phase = Phase::PlacingArch;
      break;
    case PlacingArch:
      phase = Phase::SwitchWeight;
      break;
    }
    phaseStart = time;
    setSimModelBase();
    updateStartingPos();
  }

  void ToeHeelWalk::setSimModelBase()
  {
    Leph::InverseKinematics inv(simModel);
    static std::vector<std::string> baseDOFs = {"trunk_x",
                                                "trunk_y",
                                                "trunk_z",
                                                "trunk_pitch",
                                                "trunk_roll",
                                                "trunk_yaw"};
    for (const std::string& dof : baseDOFs) {
      inv.addDOF(dof);
    }
    inv.addTargetPosition("Base", getBase());
    inv.addTargetOrientation("Base", getBase());
    inv.targetPosition("Base") = Eigen::Vector3d::Zero();
    inv.targetOrientation("Base") =Eigen::Matrix3d::Identity();
  }

      void ToeHeelWalk::updateTargetPos()
  {
    targetPos.clear();
    targetWeights.clear();
    setTarget("trunkZ", Vector3d(0,0,trunkZ), Vector3d(0,0,1));
    setTarget("COM", Vector3d(0,0,0), Vector3d(1,1,0));
    double coeff = 1;
    if (side == "right") { coeff = -1;}
    switch(phase) {
    case Waiting:
      setTarget("left_arch_center", Vector3d(0,  feetSpacing/2,0));
      setTarget("right_arch_center", Vector3d(0, -feetSpacing/2,0));
      break;
    case SwitchWeight:
      setTarget(oppSide + "_toe_center", startPos.at(oppSide + "_toe_center"));
      break;
    case LiftingArch:
      setTarget(side + "_toe_center",
                startPos.at(side + "_toe_center") + Vector3d(0,0,stepHeight));
      break;
    case FlyingFoot:
      setTarget(side + "_toe_center", Vector3d(stepX, coeff * feetSpacing, stepHeight));
      break;
    case PlacingHeel:
      setTarget(side + "_heel", Vector3d(stepX, coeff * feetSpacing, 0));
      setTarget("COM", startPos.at(oppSide + "_toe_center"));
      break;
    case PlacingArch:
      setTarget(side + "_heel", startPos.at(side + "_heel"));
      break;
    }
  }

  std::string ToeHeelWalk::getBase() {
    switch(phase) {
    case Waiting:
      return "origin";
    case SwitchWeight:
      return side    + "_arch_center";
    case LiftingArch:
      return oppSide + "_arch_center";
    case FlyingFoot:
      return oppSide + "_arch_center";
    case PlacingHeel:
      return oppSide + "_heel";
    case PlacingArch:
      return oppSide + "_toe";
    }
    throw std::logic_error("Unknown phase in ToeHeelWalk");
  }

  void updateStartingPos()
  {
    startPos.clear();
    // Due to the error of asserv in RX motors, we can't use the real starting
    // pos, therefore, we assume the targetPos of lastPhase were reach
    startPos["trunkZ"] = Vector3d(0,0,trunkZ);
    startPos["COM"]    = Vector3d(0,0,0);
    double coeff = 1;
    if (side == "right") { coeff = -1;}
    Vector3d archCenter = simModel.position(side + "_arch_center", "origin");
    Vector3d toeCenter  = simModel.position(side + "_toe_center", "origin");
    double archCenter2ToeCenter = (archCenter - toeCenter).stableNorm();
    switch(lastPhase) {
    case Waiting:
      break;
    case SwitchWeight:
      startPos[oppSide + "_toe_center"] = Vector3d(-stepX + archCenter2ToeCenter,
                                                   coeff * feetSpacing,
                                                   0);
      if (lastPhase == Phase::Waiting) {//Special case, coming from start posture
        startPos[oppSide + "_toe_center"] = Vector3d(archCenter2ToeCenter,
                                                     coeff * feetSpacing,
                                                     0);
      }
      break;
    case LiftingArch:
      setTarget(side + "_toe_center",
                startPos.at(side + "_toe_center") + Vector3d(0,0,stepHeight));
      break;
    case FlyingFoot:
      setTarget(side + "_toe_center", Vector3d(stepX, coeff * feetSpacing, stepHeight));
      break;
    case PlacingHeel:
      setTarget(side + "_heel", Vector3d(stepX, coeff * feetSpacing, 0));
      setTarget("COM", startPos.at(oppSide + "_toe_center"));
      break;
    case PlacingArch:
      setTarget(side + "_heel", startPos.at(side + "_heel"));
      break;
    }
  }


}
