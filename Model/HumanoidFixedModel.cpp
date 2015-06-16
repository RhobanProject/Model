#include "Model/HumanoidFixedModel.hpp"

namespace Leph {

HumanoidFixedModel::HumanoidFixedModel(
    RobotType type) :
    _supportFoot(LeftSupportFoot),
    _modelLeft(type, "left_foot_tip"),
    _modelRight(type, "right_foot_tip")
{
}
        
HumanoidFixedModel::SupportFoot HumanoidFixedModel::
    getSupportFoot() const
{
    return _supportFoot;
}
        
void HumanoidFixedModel::setSupportFoot(SupportFoot foot)
{
    _supportFoot = foot;
}
        
const HumanoidModel& HumanoidFixedModel::get() const
{
    if (_supportFoot == LeftSupportFoot) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}
HumanoidModel& HumanoidFixedModel::get()
{
    if (_supportFoot == LeftSupportFoot) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}
        
void HumanoidFixedModel::updateBase()
{
    //Check if moving foot is touching
    //and switch current model with 
    //new supporting foot
    if (_supportFoot == LeftSupportFoot) {
        Eigen::Vector3d posFoot = 
            _modelLeft.position("right_foot_tip", "origin");
        if (posFoot.z() < 0.0) {
            Eigen::Matrix3d rotation = 
                _modelLeft.orientation("right_foot_tip", "origin").transpose();
            _modelRight.importDOF(_modelLeft);
            _modelRight.setDOF("base_x", posFoot.x());
            _modelRight.setDOF("base_y", posFoot.y());
            _modelRight.setDOF("base_yaw", atan2(rotation(1, 0), rotation(0, 0)));
            _supportFoot = RightSupportFoot;
        }
    } else {
        Eigen::Vector3d posFoot = 
            _modelRight.position("left_foot_tip", "origin");
        if (posFoot.z() < 0.0) {
            Eigen::Matrix3d rotation = 
                _modelRight.orientation("left_foot_tip", "origin").transpose();
            _modelLeft.importDOF(_modelRight);
            _modelLeft.setDOF("base_x", posFoot.x());
            _modelLeft.setDOF("base_y", posFoot.y());
            _modelLeft.setDOF("base_yaw", atan2(rotation(1, 0), rotation(0, 0)));
            _supportFoot = LeftSupportFoot;
        }
    }
}
        
void HumanoidFixedModel::setOrientation(
    double trunkPitch, double trunkRoll)
{
    //Trunk euler angle orientation to 
    //rotation matrix
    Eigen::AngleAxisd pitchRot(-trunkPitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollRot(-trunkRoll, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> quat = rollRot*pitchRot;
    Eigen::Matrix3d baseToTrunk = quat.matrix();
    
    //Computing rotation matrix from support foot tip to trunk
    Eigen::Matrix3d footToTrunk;
    if (_supportFoot == LeftSupportFoot) {
        footToTrunk = _modelLeft.orientation("left_foot_tip", "trunk");
    } else {
        footToTrunk = _modelRight.orientation("right_foot_tip", "trunk");
    }
    
    //Computing rotation matrix to apply on floating base
    //from base to foot
    Eigen::Matrix3d baseToFoot = footToTrunk * baseToTrunk;
    
    //Retrieve euler angles from rotation matrix
    //(Manual computing without singular check seems better than
    //Eigen euler angles and with better range)
    Eigen::Vector3d angles;
    angles(0) = atan2(baseToFoot(1, 2), baseToFoot(2, 2));
    angles(1) = atan2(-baseToFoot(0, 2), 
        sqrt(baseToFoot(0,0)*baseToFoot(0,0) + baseToFoot(0,1)*baseToFoot(0,1)));
    angles(2) = atan2(baseToFoot(0, 1), baseToFoot(0, 0));

    //Assign floating base DOFs with fixed yaw
    if (_supportFoot == LeftSupportFoot) {
        _modelLeft.setDOF("base_roll", angles(0));
        _modelLeft.setDOF("base_pitch", angles(1));
    } else {
        _modelRight.setDOF("base_roll", angles(0));
        _modelRight.setDOF("base_pitch", angles(1));
    }
}

Eigen::Vector3d HumanoidFixedModel::zeroMomentPoint(
    const std::string& frame,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    Eigen::VectorXd torque = get().inverseDynamics(
        velocity, acceleration);

    double Fx = torque(get().getDOFIndex("base_x"));
    double Fy = torque(get().getDOFIndex("base_y"));
    double Fz = torque(get().getDOFIndex("base_z"));
    double MRoll = torque(get().getDOFIndex("base_roll"));
    double MPitch = torque(get().getDOFIndex("base_pitch"));
    
    double Mx;
    double My;
    convertFootMoment(MPitch, MRoll, Mx, My);
    Eigen::Vector3d zmp = computeZMP(Fx, Fy, Fz, Mx, My);

    return get().position("origin", frame, zmp);
}
Eigen::Vector3d HumanoidFixedModel::zeroMomentPoint(
    const std::string& frame,
    const VectorLabel& velocity,
    const VectorLabel& acceleration)
{
    VectorLabel torque = get().inverseDynamics(
        velocity, acceleration);

    double Fx = torque("base_x");
    double Fy = torque("base_y");
    double Fz = torque("base_z");
    double MRoll = torque("base_roll");
    double MPitch = torque("base_pitch");
    
    double Mx;
    double My;
    convertFootMoment(MPitch, MRoll, Mx, My);
    Eigen::Vector3d zmp = computeZMP(Fx, Fy, Fz, Mx, My);
    
    return get().position("origin", frame, zmp);
}

void HumanoidFixedModel::convertFootMoment(
    double torquePitch, double torqueRoll,
    double& Mx, double& My)
{
    //Compute matrix rotation from world origin
    //to foot
    Eigen::Matrix3d mat;
    if (_supportFoot == LeftSupportFoot) {
        mat = get().orientation("left_foot_tip", "origin");
    } else {
        mat = get().orientation("right_foot_tip", "origin");
    }
    mat.transposeInPlace();

    //Compute Pitch/Roll torque vector in world frame
    Eigen::Vector3d uRoll = mat.col(0);
    Eigen::Vector3d uPitch = mat.col(1);
    Eigen::Vector3d torque = torqueRoll*uRoll + torquePitch*uPitch;

    //Retrieve X/Y component
    Mx = torque.x();
    My = torque.y();
}
        
Eigen::Vector3d HumanoidFixedModel::computeZMP(
    double Fx, double Fy, double Fz, 
    double Mx, double My)
{
    Eigen::Vector3d posA;
    if (_supportFoot == LeftSupportFoot) {
        posA = get().position("left_foot_tip", "origin");
    } else {
        posA = get().position("right_foot_tip", "origin");
    }

    double Ax = posA.x();
    double Ay = posA.y();
    double Az = posA.z();

    double Px = (-My - (Az*Fx-Ax*Fz))/Fz;
    double Py = (Mx + (Ay*Fz-Az*Fy))/Fz;
    double Pz = 0.0;

    return Eigen::Vector3d(Px, Py, Pz);
}

}

