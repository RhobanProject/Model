#pragma once

#include "Model/InverseKinematics.hpp"

//TODO inherit in Code to provide a RhIO interface

namespace Leph {

  class ToeHeelWalk {
  public:

    /**
     * Transition principe:
     * In order to smooth the transition between the phases, it is interesting
     * to register the initial position of the foot, it also allows to reduce
     * the production of lateral forces while the robot has multiple contacts
     * on the ground. However, due to the control error of RX motors, we cannot
     * base our starting position on the measured position, because this would
     * mean a sudden change of orders on a 'flying body' and it would reduce
     * suddenly the targeted feetSpacing. Therefore the choice is the following:
     * - For start positions:
     *   - For frames which are grounded at the beginning of the phase, use the
     *     measured position
     *   - For frames flying at the beginning of the phase, use the theoric
     *     position
     * - For target positions:
     *   - If the target is grounded during the whole phase, do not change its
     *     position respectively to the base during the whole process
     *   - If the target is flying during a part of the phase, then use the
     *     theoric position
     * This policy will still let some sudden changes in order appears, but only
     * during double support phases which are more stable.
     * - trunkZ and  will always be subject to static error, therefore, the trunk
     *   target is always set to its theoric position
     * - The center of mass is subject to external control, therefore the target
     *   shouldn't change suddenly and it always needs to be set from theoric
     *   value.
     *
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
     * - For each phase, phaseRatio is specified as following
     *   - given an elapsed time in phase 't' and a phase duration of 'phaseT':
     *     phaseRatio(t) = min(1, t / phaseT)
     * - For each phase, the target at 't' is specified as following
     *   - target * phaseRatio(t) + start * (1 - phaseRatio(t))
     *
     * For all phases, unless something else is specified:
     * - TrunkZ at (~,~,trunkZ)
     * - COM at (0,0,~)
     * - A_toe = 0
     * - B_toe = 0
     * - base at (0,0,0)
     * - base Orientation: Identity
     *
     * Waiting: (based in origin)
     * - no constraint on base
     * - Both arch_center are at (0, -feetSpacing/2, 0) and (0, feetSpacing/2, 0)
     * - Orientation of both arch_center and trunk are Identity
     *
     * PlacingHeelA: (based in B_heel)
     * - A_heel at (stepX, +-feetSpacing, 0)
     * - orientation of A_heel is rotY(-landingHeelAngle)
     * - COM: at start(B_toe_center) with z ignored
     *
     * PlacingArchA: (based in B_toe)
     * - B_toe =  -maxToeAngle * phaseRatio
     * - A_heel at start(A_heel)
     * - orientation of A_heel is rotY(-landingHeelAngle * (1 - phaseRatio(t)))
     *
     * SwitchWeightA: (based in A_arch_center) send weight to foot A
     * - B_toe =  maxToeAngle
     * - B_toe_center at start(B_toe_center)
     * - orientation of B_toe_center is Identity
     *
     * LiftingToeA: (based in B_arch_center)
     * - A_toe_angle = maxToeAngle * (1 - phaseRatio)
     * - A_toe_center = (-stepX + archCenter2ToeCenter(B), +-feetSpacing, stepHeight)
     * - Orientation at A_toe_center = Identity
     *
     * FlyingFootA: (based in B_arch_center)
     * - A_heel at (stepX - archCenter2Heel, +- feetSpacing, stepHeight)
     * - orientation of A_heel is rotY(-landingHeelAngle * phaseRatio(t))
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
      Waiting,//After Waiting phas, jump to switchWeight with side = right
      PlacingHeel,
      PlacingArch,
      SwitchWeight,
      LiftingToe,
      FlyingFoot
    };

    static const std::string& getName(enum Phase p);

  protected:

    // Until the walk has been 'fed' with a Model, it is not properly initialized
    bool initialized;
    // When did last phase change occur?
    double phaseStart;
    enum Phase phase, lastPhase;

    // WALK PARAMETERS
    double trunkZ;//[m]
    double feetSpacing;//[m]
    double stepHeight;//[m]

    // Toe and heel angles [rad]
    double maxToeAngle;
    double landingHeelAngle;

    // Phase expected durations [s]
    double placingHeelTime;
    double placingArchTime;
    double switchWeightTime;
    double liftingArchTime;
    double flyingFootTime;


    double stepX;
    //include stepY and stepTheta later

    // Targets position at phase start
    std::map<std::string, Eigen::Vector3d> startPos;
    // Wished positions at end of the phase and weights of the targets
    std::map<std::string, Eigen::Vector3d> targetPos;
    std::map<std::string, Eigen::Vector3d> targetWeights;

    // Model used for simulation
    Leph::Model simModel;
    
    std::string side;
    std::string oppSide;

    void setTarget(const std::string& frameName,
                   const Eigen::Vector3d& position,
                   const Eigen::Vector3d& weights = Eigen::Vector3d(1,1,1));

    void swapSide();

    // Return base corresponding to current phase
    std::string getBase();

    void setSimModelBase();
    void updateStartingPos();
    void updateTargetPos();

    // Phase ratio in [0,1]
    double getPhaseRatio(double time);

    /**
     * Sometimes those distances are required
     */
    // distance depends on toeAngle of simModel
    double archCenter2ToeCenter(const std::string & side);
    // no side required since the distance is fixed
    double archCenter2Heel();

  public:
    ToeHeelWalk();

    // Also set the model m angles for toes. ik should be clean.
    void initIK(Model & m, InverseKinematics & ik, double time);

    void nextPhase(const Model& m, double time);

    std::string getPhaseName() const;

    // Return a list of the gauges which are supposed to be in contact
    std::vector<std::string> expectedPressures();

  };
}
