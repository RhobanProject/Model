#pragma once

#include "Model/InverseKinematics.hpp"

namespace Leph {

  class ToeHeelWalk {
  private:

    /**
     * Description of the phases:
     * - Except Waiting phases, all phases are symetrical, therefore when
     *   described, we use A for the current foot and B for the opposite
     * - TrunkZ stay static at all time
     * - FootSpacing stay static at all time (until stepY is introduced)
     * - Each step use a given referential
     * - When speaking about orientation, unless something explicitly say the
     *   opposite, the orientation is defined compared to origin
     * - The orientation of the base used is always Identity()
     * - When a constraint along an axis is not used, it is marqued by ~
     * - When the sign of a value depends on the foot it is written '+-value'
     * - For eache phase, phaseRatio is specified as following
     *   - given an elapsed time in phase 't' and a phase duration of 'phaseT':
     *     phaseRatio(t) = min(1, t / phaseT)
     * - For each phase, the target at 't' is specified as following
     *   - target * phaseRatio(t) + start * (1 - phaseRatio(t))
     *
     * For all phases, unless something else is specified:
     * - TrunkZ at (~,~,trunkZ)
     * - A_toe = 0
     * - B_toe = 0
     *
     * Waiting: (based in origin)
     * - Both arch_center are at (0, -footSpacing/2, 0) and (0, footSpacing/2, 0)
     * - Toes are 0
     * - COM at (0,0,~)
     * - Orientation of both arch_center and trunk are Identity
     *
     * PlacingHeelA: (based in B_heel)
     * - A_heel at (stepX, +-feetSpacing, 0)
     * - orientation of A_heel is rotY(-landingHeelAngle)
     * - COM: at B_toe_center with z ignored
     *
     * LiftingHeelA: (based in A_toe) might also be called PlaceArchB
     * - A_toe =  maxToeAngle * phaseRatio
     * - B_heel at start(B_heel)
     * - orientation of B_heel is rotY(-landingHeelAngle * (1 - phaseRatio(t)))
     * - COM at (0, 0,~)
     *
     * SwitchWeightA: (based in A_arch_center) send weight to foot A
     * - B_toe =  maxToeAngle
     * - B_toe_center at start(B_toe_center)
     * - orientation of B_toe_center is Identity
     * - COM at (0,0,0)
     *
     * LiftingArchA: (based in B_arch_center)
     * - A_toe_angle = maxToeAngle * (1 - phaseRatio)
     * - A_toe_center = start(A_toe_center) + (0,0,stepHeight)
     * - Orientation at A_toe_center = Identity
     * - COM at (0,0,0)
     *
     * FlyingFootA: (based in B_arch)
     * - A_arch_center at (stepX, +- footSpacing, stepHeight)
     * - orientation of A_heel is rotY(-landingHeelAngle * phaseRatio(t))
     * - COM at (0,0,0)
     *
     * Summary of possible targets:
     * - COM x,y
     * - TrunkZ
     * - For each foot
     *   - toe_center  (with orientation)
     *   or
     *   - arch_center (with orientation)
     *   or
     *   - heel (with orientation)
     * DOFS:
     * - Base
     * - All legs (except toe)
     */
    enum Phase {
      Waiting,
      PlacingHeel,
      LiftingHeel,
      SwitchWeight,
      LiftingArch,
      FlyingFoot,
    };

    static std::vector<enum Phase> phases;


    // Until the walk has been fed with a Model, it is not properly initialized
    bool initialized;
    // When did last phase change occur?
    double phaseStart;
    // Phase expected durations [s]
    double placingHeelTime;
    double liftingHeelTime;
    double switchWeightTime;
    double liftingFootTime;
    double flyingFootTime;

    double stepX;
    //include stepY and stepTheta later

    // Targets position at phase start
    std::map<std::string, Eigen::Vector3d> startPos;
    // Wished positions at end of the phase
    std::map<std::string, Eigen::Vector3d> targetPos;
    
    

  public:
    ToeHeelWalk();

    //TODO bind to rhio with prefix?

    void updateModel(Model & m);
    void setTargets(InverseKinematics & ik);
    //TODO think about how correction is applied
    
  };
}
