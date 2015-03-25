#include "Model/SigmabanFloatingModel.hpp"

namespace Leph {

SigmabanFloatingModel::SigmabanFloatingModel() :
    SigmabanModel("ROOT"),
    _supportFoot(LeftSupportFoot),
    _statePosX(0.0),
    _statePosY(0.0),
    _stateRotYaw(0.0)
{
    //Center initial position of the model
    _statePosY = Model::position(
        "left foot tip", "origin").y();
}
 
SigmabanFloatingModel::SupportFoot SigmabanFloatingModel::
    getSupportFoot() const
{
    return _supportFoot;
}
        
std::string SigmabanFloatingModel::supportFootName() const
{
    if (_supportFoot == LeftSupportFoot) {
        return "left foot tip";
    } else if (_supportFoot == RightSupportFoot) {
        return "right foot tip";
    } else {
        return "";
    }
}
std::string SigmabanFloatingModel::movingFootName() const
{
    if (_supportFoot == LeftSupportFoot) {
        return "right foot tip";
    } else if (_supportFoot == RightSupportFoot) {
        return "left foot tip";
    } else {
        return "";
    }
}

void SigmabanFloatingModel::putOnGround()
{
    putSupportFootFlat();
    putSupportFootOrigin();
    findSupportFoot();
}
        
void SigmabanFloatingModel::findSupportFoot()
{
    //Feet position in trunk frame
    Eigen::Vector3d posLeftFoot = 
        Model::position("left foot tip", "origin");
    Eigen::Vector3d posRightFoot = 
        Model::position("right foot tip", "origin");
    
    //Select the lowest foot in trunk frame
    bool swapSupportFoot = false;
    if (posLeftFoot.z() < posRightFoot.z()) {
        if (_supportFoot == RightSupportFoot) {
            swapSupportFoot = true;
        }
        _supportFoot = LeftSupportFoot;
    } else {
        if (_supportFoot == LeftSupportFoot) {
            swapSupportFoot = true;
        }
        _supportFoot = RightSupportFoot;
    }
    
    //If support foot is changed, the target position
    //of new support foot (which must be fixed in world frame)
    //is update with last position and orientation of 
    //old moving foot (same foot)
    if (swapSupportFoot) {
        //Moving foot position and orientation in world frame
        Eigen::Vector3d posFootWorld = 
            Model::position(supportFootName(), "origin");
        Eigen::Matrix3d rotationWorld = 
            Model::orientation(supportFootName(), "origin").transpose();
        //Updating state
        _statePosX = posFootWorld.x();
        _statePosY = posFootWorld.y();
        _stateRotYaw = atan2(rotationWorld(1, 0), rotationWorld(0, 0));
    }
}
        
void SigmabanFloatingModel::putSupportFootFlat()
{
    //Transformation matrix expressing support foot 
    //unit vector into trunk frame
    Eigen::Matrix3d rotation = 
        Model::orientation(supportFootName(), "trunk");
    //Apply rotation arround Z world for integrating yaw rotation
    rotation = 
        Eigen::AngleAxisd(_stateRotYaw, Eigen::Vector3d(0.0, 0.0, 1.0))
        .toRotationMatrix()*rotation;
    //Convertion of this rotation matrix into roll, pitch, yaw
    //euler angles
    Eigen::Vector3d angles = rotation.eulerAngles(0, 1, 2);
    //Updating floating joint roll, pitch, yaw
    Model::setDOF("base roll", angles(0));
    Model::setDOF("base pitch", angles(1));
    Model::setDOF("base yaw", angles(2));
}
        
void SigmabanFloatingModel::putSupportFootOrigin()
{
    //Support foot position in trunk frame
    Eigen::Vector3d posFoot = Model::position(supportFootName(), "trunk");
    //Convertion of TF vector (T := trunk base origin, F := support foot tip)
    //from trunk frame to world frame
    Eigen::Matrix3d rotation = Model::orientation("origin", "trunk");
    posFoot = rotation*posFoot;
    //Apply translations
    Model::setDOF("base Tx", -posFoot.x() + _statePosX);
    Model::setDOF("base Ty", -posFoot.y() + _statePosY);
    Model::setDOF("base Tz", -posFoot.z());
}

}

