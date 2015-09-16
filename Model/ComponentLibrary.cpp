#include "ComponentLibrary.hpp"

namespace Leph {
  std::map<std::string, RBDL::Body> ComponentLibrary::bodies;
  RBDL::Joint ComponentLibrary::roll (RBDL::JointTypeRevolute, Eigen::Vector3d(1,0,0));
  RBDL::Joint ComponentLibrary::pitch(RBDL::JointTypeRevolute, Eigen::Vector3d(0,1,0));
  RBDL::Joint ComponentLibrary::yaw  (RBDL::JointTypeRevolute, Eigen::Vector3d(0,0,1));
  RBDL::Joint ComponentLibrary::floatingBase(
    RBDLMath::SpatialVector(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    RBDLMath::SpatialVector(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    RBDLMath::SpatialVector(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    RBDLMath::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    RBDLMath::SpatialVector(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    RBDLMath::SpatialVector(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    );

  void ComponentLibrary::createBodies()
  {
    bodies["virtual"].mMass = 0.001;//RBDL automatically join bodies when using a fixed joint and join cannot be operatoed on two bodies with a 0 mass + Model virtual body name not implemented
    bodies["virtual"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["virtual"].mInertia = Eigen::Matrix3d::Identity();
    bodies["virtual"].mIsVirtual = true;
    bodies["EX106+"].mMass = 0.154;
    bodies["EX106+"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["EX106+"].mInertia = Eigen::Matrix3d::Identity();
    bodies["EX106+"].mIsVirtual = false;
    bodies["RX64"].mMass = 0.125;
    bodies["RX64"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["RX64"].mInertia = Eigen::Matrix3d::Identity();
    bodies["RX64"].mIsVirtual = false;
    bodies["RX28"].mMass = 0.072;
    bodies["RX28"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["RX28"].mInertia = Eigen::Matrix3d::Identity();
    bodies["RX28"].mIsVirtual = false;
    bodies["ArchPlate"].mMass = 0.350;//TODO measure
    bodies["ArchPlate"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["ArchPlate"].mInertia = Eigen::Matrix3d::Identity();
    bodies["ArchPlate"].mIsVirtual = false;
    bodies["ToePlate"].mMass = 0.150;///TODO measure
    bodies["ToePlate"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["ToePlate"].mInertia = Eigen::Matrix3d::Identity();
    bodies["ToePlate"].mIsVirtual = false;
    bodies["pressure"].mMass = 0.002;///TODO measure
    bodies["pressure"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["pressure"].mInertia = Eigen::Matrix3d::Identity();
    bodies["pressure"].mIsVirtual = false;
  }

  const RBDL::Body ComponentLibrary::getBody(const std::string & name)
  {
    if (bodies.size() == 0) { createBodies(); }
    return bodies.at(name);
  }
}
