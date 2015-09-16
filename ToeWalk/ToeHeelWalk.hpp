#pragma once

#include "Model/InverseKinematics.hpp"

namespace Leph {

  class ToeHeelWalk {
  private:
    // Indicate where the weight should be and if necessary
    enum Phase {//TODO add a warm-up phase (bothFeet to left or something like that)
      PlacingLeftHeel,
      LiftingRightHeel,
      WeightToLeft,
      LiftingRightFoot,
      FlyingRightFoot,
      PlacingRightHeel,
      LiftingLeftHeel,
      WeightToRight,
      LiftingLeftFoot,
      FlyingLeftFoot
    };


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
