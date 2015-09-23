#include "ToeHeelWalk.hpp"

#include "ModelBuilder.hpp"

using Eigen::Vector3d;

namespace Leph {
  
  ToeHeelWalk::ToeHeelWalk()
    : initialized(false), phaseStart(0),
      phase(Phase::Waiting), lastPhase(Phase::Waiting)
      placingHeelTime(2.0),
      placingArchTime(2.0),
      switchWeightTime(2.0),
      liftingFootTime(2.0),
      flyingFootTime(2.0),
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
    if (phase == Phase::Waiting) {
      inv.addTargetPosition("trunk", "trunk");
      inv.addTargetOrientation("trunk", "trunk");
      inv.targetPosition("trunk")    = Eigen::Vector3d(0,0,trunkZ);
      inv.targetOrientation("trunk") = Eigen::Matrix3d::Identity();
    }
    else {
      inv.addTargetPosition("Base", getBase());
      inv.addTargetOrientation("Base", getBase());
      inv.targetPosition("Base") = Eigen::Vector3d::Zero();
      inv.targetOrientation("Base") = Eigen::Matrix3d::Identity();
    }

    inv.run(0.0001, 50);
  }

  void ToeHeelWalk::setTarget(const std::string& frameName,
                              const Eigen::Vector3d& position,
                              const Eigen::Vector3d& weights)
  {
    targetPos[frameName] = position;
    targetWeights[frameName] = weights;
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
                Vector3d(archCenter2ToeCenter(oppSide) - stepX,
                         coeff * footSpacing,
                         stepHeight));
      break;
    case FlyingFoot:
      setTarget(side + "_heel", Vector3d(stepX - archCenter2Heel(),
                                         coeff * feetSpacing,
                                         stepHeight));
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

  double ToeHeelWalk::archCenter2ToeCenter(const std::string & side)
  {
    Vector3d archCenter = simModel.position(side + "_arch_center", "origin");
    Vector3d toeCenter  = simModel.position(side + "_toe_center", "origin");
    return (archCenter - toeCenter).stableNorm();
  }

  double ToeHeelWalk::archCenter2Heel()
  {
    Vector3d archCenter = simModel.position("left_arch_center", "origin");
    Vector3d heel  = simModel.position("left_heel", "origin");
    return (archCenter - heel).stableNorm();
  }

  void ToeHeelWalk::updateStartingPos()
  {
    startPos.clear();
    // Due to the error of asserv in RX motors, we can't use the real starting
    // pos, therefore, we assume the targetPos of lastPhase were reach
    startPos["trunkZ"] = Eigen::Vector3d(0, 0, trunkZ);
    startPos["COM"] = Vector3d::Zero();//Default case, COM starts above the current phase base
    double coeff = 1;
    if (side == "right") { coeff = -1;}
    switch(lastPhase) {
    case Waiting://Current phase: switchWeight, based on target_arch_center
      startPos[side + "_toe_center"] = simModel.position(side + "_toe_center",
                                                         "origin");
      startPos["COM"] = Vector3d(0, -coeff * feetSpacing / 2, 0);
      break;
    case SwitchWeight://Current phase: Lifting Arch, based in oppSide_arch_center
      startPos[side + "_toe_center"] = simModel.position(side + "_toe_center");
      break;
    case LiftingArch://Current phase: Flying foot, based in oppSide_arch_center
      startPos[side + "_heel"] = Vector3d(-stepX - archCenter2Heel(),
                                          coeff * footSpacing,
                                          stepHeight);
      break;
    case FlyingFoot://Current phase: PlacingHeel, based in oppSide_heel
      startPos[side + "_heel"] = Vector3d(stepX,
                                          coeff * footSpacing,
                                          stepHeight);
      startPos["COM"] = Eigen::Vector3d(archCenter2Heel(), 0, 0);
      break;
    case PlacingHeel://Current phase: PlacingArch, based in oppSide_toe
      startPos[side + "_heel"] = simModel.position(side + "_heel", "origin");
      break;
    case PlacingArch://Current phase: Switch weight based in target_center
      startPos[oppSide + "_toe_center"] = simModel.position(side + "_toe_center", "origin");
      break;
    }
  }

  void ToeHeelWalk::getPhaseRatio()
  {
    double phaseT;
    switch(phase){
    case Waiting: return 1;
    case SwitchWeight: phaseT = switchWeightTime; break;
    case LiftingArch:  phaseT = liftingArchTime ; break;
    case FlyingFoot:   phaseT = flyingFootTime  ; break;
    case PlacingHeel:  phaseT = placingHeelTime ; break;
    case PlacingArch:  phaseT = placingArchTime ; break;
    }
  }

}
