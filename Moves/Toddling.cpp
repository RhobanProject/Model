#include "Toddling.hpp"

#include "Utils/STLibrary.hpp"

/**
 * from: https://www.youtube.com/watch?v=qT9qzwCJjAk (28:05 and 38:40)
 * ZMP:
 *   z_{x,y} = c_{x,y} - c_z / g_z * c''_{x,y}
 * Capture Point:
 *   ksi = c + 1/w * c' (with w the period of the pendulum)
 *
 * let a = t * freq be the phase of the movement, considered in [0,1]
 * Since walk is based on c_y = sin(2 * pi * a) * comAmplitude:
 * c'_y  =  cos(a) * amplitude * (2 * pi * freq)
 * c''_y = -sin(a) * amplitude * (2 * pi * freq)^2
 * Then, the maximum of z_y is in a = pi/2 with:
 * z_y = comAmplitude + c_z / g_z * comAmplitude * (2 * pi * freq)^2 
 * Therefore, we should directly avoid to have comAmplitude much bigger than
 * feetSpacing / (1 + c_z / g_z * (2 * pi * freq)^2)
 *
 * Remarks:
 * - While c_z/ g_z is quite small, (2* pi * freq)^2 can grow quite
 *   quickly at 'high frequency'
 * - Practically the value used are far under the value obtained from the
 *   equation
 *
 *
 * Since ksi is the destination of the zmp, it would be convenient to have it
 * directed toward the base of the opposite foot when moving toward:
 * Analysis is still to be done
 */

static double g = 9.81;


namespace Leph {
  Toddling::Toddling() : phase(0.0),
                         comZ(0.35),
                         comX(0.0),
                         comAmplitude(0.02),
                         frequency(1.2),
                         feetSpacing(0.2),
                         extraShoulderRoll(30),
                         wishedTrunkPitch(5)
  {
  }

  Eigen::Vector3d Toddling::wishedCOM() const
  {
    double posY = sin(phase * 2 * M_PI) * comAmplitude;
    return Eigen::Vector3d(comX, posY, comZ);
  }

  Eigen::Vector3d Toddling::expectedCOP() const
  {
    double accY = - sin(phase * 2 * M_PI) * std::pow(2 * M_PI * frequency, 2) * comAmplitude;
    double coeffAcc = comZ / g;
    double posY = sin(phase * 2 * M_PI) * comAmplitude;
    return Eigen::Vector3d(comX, posY - coeffAcc * accY, 0);
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
      if (dof.find("toe") != std::string::npos) {
        continue;
      }
      ik.addDOF(dof);
    }
    for (const auto& dof : m.getDOFCategory("base")) {
      ik.addDOF(dof);
    }

    // Setting extra angles on model
    std::map<std::string, int> coeffs = {{"left", 1},{"right", -1}};
    for (const auto& entry : coeffs) {
      std::string dof = entry.first + "_shoulder_roll";
      m.setDOF(dof, entry.second * extraShoulderRoll * M_PI / 180.0);
    }

    // Setting COM target
    ik.addTargetCOM();
    ik.targetCOM() = wishedCOM();

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
    ik.targetOrientation("trunk") = rotY(wishedTrunkPitch * M_PI /  180);
    ik.weightOrientation("trunk") = 0.01;

    // Light constraint and Bound on knee
    for (const std::string& side : {"left", "right"}) {
      std::string dof = side + "_knee";
      ik.addTargetDOF(dof,dof);
      ik.targetDOF(dof) = 15 * M_PI / 180;
      ik.weightDOF(dof) = 0.001;
      ik.setLowerBound(dof, 0);
    }
  }
}
