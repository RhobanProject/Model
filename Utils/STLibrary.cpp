#include "STLibrary.hpp"

namespace Leph {

  Eigen::Matrix3d rotX(double angleRad)
  {
    double c = cos(angleRad);
    double s = sin(angleRad);
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m(1,1) =  c;
    m(1,2) = -s;
    m(2,1) =  s;
    m(2,2) =  c;
    return m;
  }

  Eigen::Matrix3d rotY(double angleRad)
  {
    double c = cos(angleRad);
    double s = sin(angleRad);
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m(0,0) =  c;
    m(0,2) =  s;
    m(2,0) = -s;
    m(2,2) =  c;
    return m;
  }

  Eigen::Matrix3d rotZ(double angleRad)
  {
    double c = cos(angleRad);
    double s = sin(angleRad);
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m(0,0) =  c;
    m(0,1) = -s;
    m(1,0) =  s;
    m(1,1) =  c;
    return m;
  }

  RBDLMath::SpatialTransform stRotX(double angleRad)
  {
    return RBDLMath::SpatialTransform(rotX(angleRad), Eigen::Vector3d::Zero());
  }
  RBDLMath::SpatialTransform stRotY(double angleRad)
  {
    return RBDLMath::SpatialTransform(rotY(angleRad), Eigen::Vector3d::Zero());
  }
  RBDLMath::SpatialTransform stRotZ(double angleRad)
  {
    return RBDLMath::SpatialTransform(rotZ(angleRad), Eigen::Vector3d::Zero());
  }

}
