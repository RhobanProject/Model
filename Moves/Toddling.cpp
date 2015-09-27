#include "Toddling.hpp"

namespace Leph {
  Toddling::Toddling() : phase(0.0),
                         comZ(0.3),
                         comAmplitude(0.06),
                         frequency(1.0),
                         feetSpacing(0.2)
  {
  }

  void Toddling::update(double elapsed)
  {
    phase += elapsed * frequency;
    if (phase > 1) {
      phase -= (int)phase;
    }
  }

  void Toddling::initIK(Model& m, InverseKinematics& ik)
  {
    for (const auto& dof : m.getDOFCategory("lowerBody")) {
      ik.addDOF(dof);
    }
    for (const auto& dof : m.getDOFCategory("base")) {
      ik.addDOF(dof);
    }

    // Setting COM target
    ik.addTargetCOM();
    ik.targetCOM() = Eigen::Vector3d(0, sin(phase * 2 * M_PI) * comAmplitude, comZ);

    // Setting Foot target
    std::map<std::string, int> sideCoeff = { {"left", 1}, {"right",-1} };
    for (const auto& side : sideCoeff) {
      std::string frameName = side.first + "_arch_center";
      // Position
      ik.addTargetPosition(frameName, frameName);
      ik.targetPosition(frameName) = Eigen::Vector3d(0, side.second * feetSpacing / 2, 0);
      // Orientation
      ik.addTargetOrientation(frameName, frameName);
      ik.targetOrientation(frameName) = Eigen::Matrix3d::Identity();
    }

    // Light constraint on torso orientation
    ik.addTargetOrientation("trunk", "trunk");
    ik.targetOrientation("trunk") = Eigen::Matrix3d::Identity();
    ik.weightOrientation("trunk") = 0.001;
  }
}
