#pragma once

#include <rbdl/rbdl.h>

// Access easily to some useful transformation matrix or spatial transform

namespace Leph {

  namespace RBDL = RigidBodyDynamics;
  namespace RBDLMath = RigidBodyDynamics::Math;


  // Return matrix corresponding to the elementary rotation given an angle in rad
  Eigen::Matrix3d rotX(double angleRad);
  Eigen::Matrix3d rotY(double angleRad);
  Eigen::Matrix3d rotZ(double angleRad);
  RBDLMath::SpatialTransform stRotX(double angleRad);
  RBDLMath::SpatialTransform stRotY(double angleRad);
  RBDLMath::SpatialTransform stRotZ(double angleRad);

}
