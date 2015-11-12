#pragma once

#include "Model/Model.hpp"
#include "Model/InverseKinematics.hpp"

namespace Leph {
  class Toddling {
  protected:
    // Current phase in [0,1]
    double phase;

    // COM movement
    double comZ;//[m]
    double comX;//[m]
    double comAmplitude;//[m]
    double frequency;//[Hz]

    // Static properties
    double feetSpacing;//[m]
    double extraShoulderRoll;//[deg]
    double wishedTrunkPitch;//[deg]

    // Step properties
    double stepHeight;//[m]
    double stepX;
    double stepY;
    double stepTheta;
    double doubleSupportRatio;//in [0,1]

    double getFootHeight(double footPhase) const;
    double getStepX(double footPhase) const;

  public:

    Toddling();

    double getPhase() const;
    double getPhase(const std::string& side) const;
    Eigen::Vector3d getFootTarget(const std::string& side) const;

    // Elapsed is in [s]
    void update(double elapsed);

    void initIK(Model & m, InverseKinematics & ik);

    Eigen::Vector3d wishedCOM() const;
    Eigen::Vector3d expectedCOP() const;
  };
}
