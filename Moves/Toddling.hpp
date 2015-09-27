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
    double comAmplitude;//[m]
    double frequency;//[Hz]

    double feetSpacing;//[m]

  public:

    Toddling();

    // Elapsed is in [s]
    void update(double elapsed);

    void initIK(Model & m, InverseKinematics & ik);
  };
}
