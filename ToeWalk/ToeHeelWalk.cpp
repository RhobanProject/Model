#include "ToeHeelWalk.hpp"

#include "Model/ModelBuilder.hpp"
#include "Utils/STLibrary.hpp"

using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace Leph {

  const std::string& ToeHeelWalk::getName(enum Phase p)
  {
    static std::map<enum Phase, std::string> phaseNames =
    { 
      { Waiting, "Waiting"},
      { PlacingHeel, "PlacingHeel"},
      { PlacingArch, "PlacingArch"},
      { SwitchWeight, "SwitchWeight"},
      { LiftingArch, "LiftingArch"},
      { FlyingFoot, "FlyingFoot"}
    };
    try{
      return phaseNames.at(p);
    }
    catch (const std::out_of_range& exc) {
      throw std::out_of_range("ToeHeelWalk: Cannot find name of requested phase");
    }
  }
  
  ToeHeelWalk::ToeHeelWalk()
    : initialized(false), phaseStart(0),
      phase(Phase::Waiting), lastPhase(Phase::Waiting),
      trunkZ(0.45), feetSpacing(0.2), stepHeight(0.15),
      maxToeAngle(60 * M_PI / 180),
      landingHeelAngle(20 * M_PI / 180),
      placingHeelTime(2.0),
      placingArchTime(2.0),
      switchWeightTime(2.0),
      liftingArchTime(2.0),
      flyingFootTime(2.0),
      stepX(0.15),
      simModel(generateGrobanWithToe(true)),
      side("none"),
      oppSide("none")
  {
    // In waiting state, source are the same as target
    updateTargetPos();
    for (const auto& targetEntry : targetPos) {
      startPos[targetEntry.first] = targetEntry.second;
    }
  }

  void ToeHeelWalk::swapSide()
  {
    std::cout << "side before swap: " << side << std::endl;
    std::string tmp = side;
    side = oppSide;
    oppSide = tmp;
    std::cout << "side after swap: " << side << std::endl;
  }

  void ToeHeelWalk::nextPhase(const Model& m, double time)
  {
    phaseStart = time;
    lastPhase = phase;
    // Set side and phase appopriately
    switch(phase) {
    case Waiting:
      phase = Phase::SwitchWeight;
      side = "right"; oppSide = "left";
      break;
    case SwitchWeight:
      phase = Phase::LiftingArch;
      std::cout << "side before swap: " << side << std::endl;
      swapSide();
      std::cout << "side after swap: " << side << std::endl;
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
    simModel.importDOF(m);
    setSimModelBase();
    updateStartingPos();
    updateTargetPos();
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
    setTarget("trunk", Vector3d(0,0,trunkZ), Vector3d(0,0,1));
    setTarget("COM", Vector3d(0,0,0), Vector3d(1,1,0));
    if (phase != Phase::Waiting) {
      setTarget(getBase(), Vector3d(0,0,0));
    }
    double coeff = 1;
    if (side == "right") { coeff = -1;}
    switch(phase) {
    case Waiting:
      setTarget("left_arch_center" , Vector3d(0,  feetSpacing/2,0));
      setTarget("right_arch_center", Vector3d(0, -feetSpacing/2,0));
      break;
    case SwitchWeight:
      setTarget(oppSide + "_toe_center", startPos.at(oppSide + "_toe_center"));
      break;
    case LiftingArch:
      setTarget(side + "_toe_center",
                Vector3d(archCenter2ToeCenter(oppSide) - stepX,
                         coeff * feetSpacing,
                         stepHeight));
      break;
    case FlyingFoot:
      setTarget(side + "_heel", Vector3d(stepX - archCenter2Heel(),
                                         coeff * feetSpacing,
                                         stepHeight));
      break;
    case PlacingHeel:
      setTarget(side + "_heel", Vector3d(stepX, coeff * feetSpacing, 0));
      setTarget("COM", simModel.position(oppSide + "_toe_center", "origin"),
                Vector3d(1,1,0));
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
      return oppSide + "_toe_center";
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
    startPos["trunk"] = Eigen::Vector3d(0, 0, trunkZ);
    startPos["COM"] = Vector3d::Zero();//Default case, COM starts above the current phase base
    if (phase != Phase::Waiting) {
      startPos[getBase()] = Vector3d(0,0,0);
    }
    double coeff = 1;
    if (side == "right") { coeff = -1;}
    switch(lastPhase) {
    case Waiting://Current phase: switchWeight, based on target_arch_center
      std::cout << "W: Adding target '" << oppSide + "_toe_center' to startPos" << std::endl;
      startPos[oppSide + "_toe_center"] = simModel.position(oppSide + "_toe_center",
                                                            "origin");
      startPos["COM"] = Vector3d(0, -coeff * feetSpacing / 2, 0);
      break;
    case SwitchWeight://Current phase: Lifting Arch, based in oppSide_arch_center
      std::cout << "SW: Adding target '" << side + "_toe_center' to startPos" << std::endl;
      startPos[side + "_toe_center"] = simModel.position(side + "_toe_center", "origin");
      break;
    case LiftingArch://Current phase: Flying foot, based in oppSide_arch_center
      startPos[side + "_heel"] = Vector3d(-stepX - archCenter2Heel(),
                                          coeff * feetSpacing,
                                          stepHeight);
      break;
    case FlyingFoot://Current phase: PlacingHeel, based in oppSide_heel
      startPos[side + "_heel"] = Vector3d(stepX,
                                          coeff * feetSpacing,
                                          stepHeight);
      startPos["COM"] = Eigen::Vector3d(archCenter2Heel(), 0, 0);
      break;
    case PlacingHeel://Current phase: PlacingArch, based in oppSide_toe
      startPos[side + "_heel"] = simModel.position(side + "_heel", "origin");
      break;
    case PlacingArch://Current phase: Switch weight based in target_center
      startPos[oppSide + "_toe_center"] = simModel.position(oppSide + "_toe_center", "origin");
      startPos["COM"] = simModel.centerOfMass("origin");
      break;
    }
  }

  double ToeHeelWalk::getPhaseRatio(double time)
  {
    double phaseT(0);
    switch(phase){
    case Waiting: return 1;
    case SwitchWeight: phaseT = switchWeightTime; break;
    case LiftingArch:  phaseT = liftingArchTime ; break;
    case FlyingFoot:   phaseT = flyingFootTime  ; break;
    case PlacingHeel:  phaseT = placingHeelTime ; break;
    case PlacingArch:  phaseT = placingArchTime ; break;
    }
    if (phaseT == 0) { throw std::logic_error("Unknown phase for ToeHeelWalk");}
    double dt = time - phaseStart;
    return std::min(1.0, dt / phaseT);
  }

  void ToeHeelWalk::initIK(Model & m, InverseKinematics & ik, double time)
  {
    // ADDING DOF
    std::vector<std::string> sides = {"left", "right"};
    std::vector<std::string> dofs = {"hip_roll", "hip_yaw", "hip_pitch",
                                     "knee",
                                     "ankle_roll", "ankle_pitch"};
    for (const auto& side : sides) {
      for (const auto& dof : dofs) {
        ik.addDOF(side + "_" + dof);
      }
    }
    std::vector<std::string> baseDofs = {"trunk_x",
                                         "trunk_y",
                                         "trunk_z",
                                         "trunk_roll",
                                         "trunk_pitch",
                                         "trunk_yaw"};
    for (const auto& dof : baseDofs) {
        ik.addDOF(dof);
    }

    ik.setLowerBound("left_knee", 15 * M_PI / 180);
    ik.setLowerBound("right_knee", 15 * M_PI / 180);
    ik.setUpperBound("left_knee", M_PI);
    ik.setUpperBound("right_knee",M_PI);

    double pRatio = getPhaseRatio(time);
    // SETTING POSITION TARGETS
    for (const auto& targetEntry : targetPos) {
      const std::string& targetName = targetEntry.first;
      Vector3d realTarget;
      try{
        const Vector3d& target = targetEntry.second;
        const Vector3d& src = startPos.at(targetName);
        realTarget = pRatio * target + (1 - pRatio) * src;
      }
      catch(const std::out_of_range& exc) {
        throw std::logic_error("ToeHeelWalk: no start pos for '" + targetName + "'");
      }
      std::cout << "setting target '" << targetName << "' at:" << realTarget.transpose() << std::endl;

      if (targetName == "COM") {
        ik.addTargetCOM();
        ik.targetCOM() = realTarget;
        ik.weightCOM() = targetWeights.at(targetName);
      }
      else {
        ik.addTargetPosition(targetName, targetName);
        ik.targetPosition(targetName) = realTarget;
        ik.weightPosition(targetName) = targetWeights.at(targetName);
      }
    }
    // SETTING ORIENTATION TARGETS
    // base always heading oriented as origin
    ik.addTargetOrientation("base", getBase());
    ik.targetOrientation("base") = Matrix3d::Identity();
    // Also forcing trunk to be well oriented, otherwise it can do anything stupid
    ik.addTargetOrientation("trunk", "trunk");
    ik.targetOrientation("trunk") = Matrix3d::Identity();

    // Other orientation constraint differs on the phase
    switch(phase) {
    case Waiting:
      for (const auto& side: sides) {
        ik.addTargetOrientation(side + "_arch_center", side + "_arch_center");
        ik.targetOrientation(side + "_arch_center") = Matrix3d::Identity();
      }
      break;
    case SwitchWeight:
      ik.addTargetOrientation(oppSide + "_toe_center", oppSide + "_toe_center");
      ik.targetOrientation(oppSide + "_toe_center") = Matrix3d::Identity();
      break;
    case LiftingArch:
      ik.addTargetOrientation(side + "_toe_center", side + "_toe_center");
      ik.targetOrientation(side + "_toe_center") = Matrix3d::Identity();
      break;
    case FlyingFoot: 
      ik.addTargetOrientation(side + "_heel", side + "_heel");
      ik.targetOrientation(side + "_heel") = rotY(-landingHeelAngle * pRatio);
      break;
    case PlacingHeel:
      ik.addTargetOrientation(side + "_heel", side + "_heel");
      ik.targetOrientation(side + "_heel") = rotY(-landingHeelAngle);
      break;
    case PlacingArch:
      ik.addTargetOrientation(side + "_heel", side + "_heel");
      ik.targetOrientation(side + "_heel") = rotY(-landingHeelAngle * (1 - pRatio));
      break;
    }
    // SETTING TOES TARGET
    m.setDOF("left_toe", 0);
    m.setDOF("right_toe", 0);
    switch(phase) {
    case PlacingArch:
      m.setDOF(oppSide + "_toe", -maxToeAngle * pRatio);
      break;
    case SwitchWeight:
      m.setDOF(oppSide + "_toe", -maxToeAngle);
      // Special warm-up case
      if (lastPhase == Phase::Waiting) {
        m.setDOF(oppSide + "_toe", -maxToeAngle * pRatio);
      }
      break;
    case LiftingArch:
      m.setDOF(side + "_toe", -maxToeAngle * (1 - pRatio));
      break;
    default: break;//Keep toe as target
    }
    // Everything has been done!
  }

  std::string ToeHeelWalk::getPhaseName() const
  {
    return getName(phase) + side;
  }

}
