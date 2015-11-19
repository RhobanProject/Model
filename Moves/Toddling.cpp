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
 * c'_y  =  cos(2 * pi * a) * amplitude * (2 * pi * freq)
 * c''_y = -sin(2 * pi * a) * amplitude * (2 * pi * freq)^2
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
                         wishedTrunkPitch(5),
                         stepHeight(0.03),
                         stepX(0.0),
                         stepY(0.0),
                         stepTheta(0),
                         doubleSupportRatio(0.5)
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

  double Toddling::getFootHeight(double footPhase) const
  {
    double liftDuration = (1.0 - doubleSupportRatio) / 2;
    double startLift = 0.75 - liftDuration / 2;
    double endLift = 0.75 + liftDuration / 2;
    if (footPhase > startLift && footPhase < endLift) {
      double internalPhase = (footPhase - startLift) / liftDuration;
      double stepRatio = (1 - cos(internalPhase * 2 * M_PI)) / 2.0;
      return stepHeight * stepRatio;
    }
    return 0;
  }

  double Toddling::getStepX(double footPhase) const
  {
    double liftDuration = (1.0 - doubleSupportRatio) / 2;
    double startLift = 0.75 - liftDuration / 2;
    double endLift = 0.75 + liftDuration / 2;
    double groundSpeed = stepX / (1 - liftDuration);//Ground speed while the foot is in contact
    double dt = footPhase;
    if (footPhase > startLift && footPhase < endLift) {
      // Flying phase
      double internalPhase = (footPhase - startLift) / liftDuration;
      dt -= startLift;
      double alpha = M_PI * (internalPhase - 0.5);//[-pi/2,pi/2]
      double uncorrected = - stepX / 2 - groundSpeed * dt;
      double correction = (sin(alpha) + 1) / 2 * (stepX + groundSpeed * liftDuration);
      return uncorrected + correction;
    }
    else if (footPhase > endLift) {
      dt -= endLift;
    }
    else {
      dt += 1 - endLift;
    }
    return stepX / 2 - dt * groundSpeed;
  }

  // Return the offset from the start position, start position is not the same for both feet
  double Toddling::getStepY(double footPhase) const
  {
    double liftDuration = (1.0 - doubleSupportRatio) / 2;
    double startLift = 0.75 - liftDuration / 2;
    double endLift = 0.75 + liftDuration / 2;
    double groundSpeed = stepY / (1 - liftDuration);//Ground speed while the foot is in contact
    double dt = footPhase;
    if (footPhase > startLift && footPhase < endLift) {
      // Flying phase
      double internalPhase = (footPhase - startLift) / liftDuration;
      dt -= startLift;
      double alpha = M_PI * (internalPhase - 0.5);//[-pi/2,pi/2]
      double uncorrected = - stepY / 2 - groundSpeed * dt;
      double correction = (sin(alpha) + 1) / 2 * (stepY + groundSpeed * liftDuration);
      return uncorrected + correction;
    }
    else if (footPhase > endLift) {
      dt -= endLift;
    }
    else {
      dt += 1 - endLift;
    }
    return stepY / 2 - dt * groundSpeed;
  }


  double Toddling::getPhase() const
  {
    return phase;
  }

  double Toddling::getPhase(const std::string& side) const
  {
    if (side == "left") {
      return phase;
    }
    else if (side == "right") {
      double rightPhase = phase + 0.5;
      if (rightPhase > 1) { rightPhase -= 1; }
      return rightPhase;
    }
    throw std::runtime_error("Toddling::getPhase(): unknown side '" + side + "'");
  }

  Eigen::Vector3d Toddling::getFootTarget(const std::string& side) const
  {
    static std::map<std::string, int> coeffs = {{"left",1},{"right",-1}};
    try{
      double staticY = coeffs.at(side) * (feetSpacing + stepY) / 2;
      Eigen::Vector3d footPos(0, staticY, 0);
      double footPhase = getPhase(side);
      footPos.x() += getStepX(footPhase);
      footPos.y() += getStepY(footPhase);
      footPos.z() += getFootHeight(footPhase);
      return footPos;
    }
    catch(const std::out_of_range& exc) {}
    throw std::out_of_range("Toddling::getFootTarget(): unknown side: '" + side + "'");
  }

  Eigen::Matrix3d Toddling::getFootOrientation(const std::string& side) const
  {
    // Time milestones
    double liftDuration = (1.0 - doubleSupportRatio) / 2;
    double footPhase = getPhase(side);
    double startOpLift = 0.25 - liftDuration / 2;
    double endOpLift   = 0.25 + liftDuration / 2;
    double startMyLift = 0.75 - liftDuration / 2;
    double endMyLift   = 0.75 + liftDuration / 2;
    // Variables
    Eigen::Matrix3d wishedOrientation = Eigen::Matrix3d::Identity();
    double theta = 0;//deg
    // Case 1: foot support alone
    if (footPhase >= startMyLift && footPhase < endMyLift) {
      double internalPhase = (footPhase - startMyLift) / liftDuration;
      theta = stepTheta/2 * cos(internalPhase * M_PI);
    }
    // Case 2: foot flying
    else if (footPhase >= startOpLift && footPhase < endOpLift) {
      double internalPhase = (footPhase - startOpLift) / liftDuration;
      theta = -stepTheta/2 * cos(internalPhase * M_PI);
    }
    // Case 3: double support 1 (post footSupport alone)
    else if (footPhase >= endOpLift && footPhase < startMyLift) {
      theta = stepTheta/2;
    }
    // Case 4: double support 2
    else {
      theta = -stepTheta/2;
    }
    wishedOrientation = rotZ(theta * M_PI / 180) * wishedOrientation;
    return wishedOrientation;
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

    // Setting 0 angles on upper body (if we don't, it slowly drifts away)
    for (const auto& dof : m.getDOFCategory("upperBody")) {
      m.setDOF(dof, 0);
    }
    // Setting extra angles on shoulderRoll
    std::map<std::string, int> coeffs = {{"left", 1},{"right", -1}};
    for (const auto& entry : coeffs) {
      std::string dof = entry.first + "_shoulder_roll";
      m.setDOF(dof, entry.second * extraShoulderRoll * M_PI / 180.0);
    }

    // Setting COM target
    ik.addTargetCOM();
    ik.targetCOM() = wishedCOM();

    // Setting Foot target
    for (const std::string& side : {"left","right"}) {
      std::string frameName = side + "_arch_center";
      // Position
      ik.addTargetPosition(frameName, frameName);
      ik.targetPosition(frameName) = getFootTarget(side);
      // Orientation
      ik.addTargetOrientation(frameName, frameName);
      ik.targetOrientation(frameName) = getFootOrientation(side);
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
