#include "Model/HumanoidFixedModel.hpp"

namespace Leph {

HumanoidFixedModel::HumanoidFixedModel(
    RobotType type) :
    _supportFoot(LeftSupportFoot),
    _modelLeft(type, "left foot tip"),
    _modelRight(type, "right foot tip")
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
            _modelLeft.position("right foot tip", "origin");
        if (posFoot.z() < 0.0) {
            Eigen::Matrix3d rotation = 
                _modelLeft.orientation("right foot tip", "origin").transpose();
            _modelRight.importDOF(_modelLeft);
            _modelRight.setDOF("base Tx", posFoot.x());
            _modelRight.setDOF("base Ty", posFoot.y());
            _modelRight.setDOF("base yaw", atan2(rotation(1, 0), rotation(0, 0)));
            _supportFoot = RightSupportFoot;
        }
    } else {
        Eigen::Vector3d posFoot = 
            _modelRight.position("left foot tip", "origin");
        if (posFoot.z() < 0.0) {
            Eigen::Matrix3d rotation = 
                _modelRight.orientation("left foot tip", "origin").transpose();
            _modelLeft.importDOF(_modelRight);
            _modelLeft.setDOF("base Tx", posFoot.x());
            _modelLeft.setDOF("base Ty", posFoot.y());
            _modelLeft.setDOF("base yaw", atan2(rotation(1, 0), rotation(0, 0)));
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
        footToTrunk = _modelLeft.orientation("left foot tip", "trunk");
    } else {
        footToTrunk = _modelRight.orientation("right foot tip", "trunk");
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
        _modelLeft.setDOF("base roll", angles(0));
        _modelLeft.setDOF("base pitch", angles(1));
    } else {
        _modelRight.setDOF("base roll", angles(0));
        _modelRight.setDOF("base pitch", angles(1));
    }
}

}

