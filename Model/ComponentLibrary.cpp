#include "ComponentLibrary.hpp"

//TODO use the real center of mass of alu parts

namespace Leph {

  // Density of the materials (kg / cm^3)
  static double aluLoicRho   = 2.7 / 1000;//Measure accurately
  //static double aluDXLRho    = 2.7 / 1000;//Measure accurately
  static double polyamideRho = 1.2 / 1000;//Measure accurately

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
    //Virtual
    bodies["Virtual"].mMass = 0.0;
    bodies["Virtual"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["Virtual"].mInertia = Eigen::Matrix3d::Identity();
    bodies["Virtual"].mIsVirtual = false;
    //Default orientation of the motors is given at files in:
    // http://en.robotis.com/BlueAD/board.php?bbs_id=downloads&mode=view&bbs_no=26324&page=1&key=&keyword=&sort=&scate=DRAWING 
    // EX106+
    bodies["EX106+"].mMass = 0.158;
    bodies["EX106+"].mCenterOfMass = Eigen::Vector3d(0.0001428, -0.01962, 0.002546);
    bodies["EX106+"].mInertia = Eigen::Matrix3d::Identity();
    bodies["EX106+"].mIsVirtual = false;
    // RX64
    bodies["RX64"].mMass = 0.1295;
    bodies["RX64"].mCenterOfMass = Eigen::Vector3d(0.0001583, -0.01702, 0.001678);
    bodies["RX64"].mInertia = Eigen::Matrix3d::Identity();
    bodies["RX64"].mIsVirtual = false;
    // RX28
    bodies["RX28"].mMass = 0.075;
    bodies["RX28"].mCenterOfMass = Eigen::Vector3d(0.0002407, -0.01290, 0.0005949);
    bodies["RX28"].mInertia = Eigen::Matrix3d::Identity();
    bodies["RX28"].mIsVirtual = false;
    // Tibia
    bodies["Tibia"].mMass = 2 * 28.468 * aluLoicRho;
    bodies["Tibia"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["Tibia"].mInertia = Eigen::Matrix3d::Identity();
    bodies["Tibia"].mIsVirtual = false;
    // Femur
    bodies["Femur"].mMass = 2 * 20 * aluLoicRho;//TODO use updated part (initial was 27.3 cm^3
    bodies["Femur"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["Femur"].mInertia = Eigen::Matrix3d::Identity();
    bodies["Femur"].mIsVirtual = false;
    // Torso
    bodies["Torso"].mMass = (16.825 + 26.989 + 17.131) * aluLoicRho;
    bodies["Torso"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["Torso"].mInertia = Eigen::Matrix3d::Identity();
    bodies["Torso"].mIsVirtual = false;
    // Arm1
    bodies["Arm1"].mMass = 12.258 * aluLoicRho;
    bodies["Arm1"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["Arm1"].mInertia = Eigen::Matrix3d::Identity();
    bodies["Arm1"].mIsVirtual = false;
    // Arm2
    bodies["Arm2"].mMass = 15.043 * aluLoicRho;
    bodies["Arm2"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["Arm2"].mInertia = Eigen::Matrix3d::Identity();
    bodies["Arm2"].mIsVirtual = false;
    // ArchPlate
    bodies["ArchPlate"].mMass = (69.940 + 34.971 + 9.673 + 21.891) * polyamideRho;
    bodies["ArchPlate"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["ArchPlate"].mInertia = Eigen::Matrix3d::Identity();
    bodies["ArchPlate"].mIsVirtual = false;
    // ToePlate
    bodies["ToePlate"].mMass = 31.806 * polyamideRho;
    bodies["ToePlate"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["ToePlate"].mInertia = Eigen::Matrix3d::Identity();
    bodies["ToePlate"].mIsVirtual = false;
    // Gauge
    bodies["Gauge"].mMass = 0.028;///TODO measure accurately
    bodies["Gauge"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["Gauge"].mInertia = Eigen::Matrix3d::Identity();
    bodies["Gauge"].mIsVirtual = false;
    // FitPC
    bodies["FitPC"].mMass = 0.350;///TODO measure accurately
    bodies["FitPC"].mCenterOfMass = Eigen::Vector3d::Zero();
    bodies["FitPC"].mInertia = Eigen::Matrix3d::Identity();
    bodies["FitPC"].mIsVirtual = false;
  }

  const RBDL::Body ComponentLibrary::getBody(const std::string & name)
  {
    if (bodies.size() == 0) { createBodies(); }
    try{
      return bodies.at(name);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("ComponentLibrary: unknown body '" + name + "'");
    } 
  }
}
