#include "Model/SigmabanModel.hpp"

namespace Leph {

SigmabanModel::SigmabanModel() :
    Model("sigmaban.urdf"),
    _supportFoot(LeftSupportFoot),
    _statePosX(0.0),
    _statePosY(0.0),
    _stateRotYaw(0.0)
{
    //Center initial position of the model
    _statePosY = Model::position(
        "left foot tip", "origin").y();
}
        
SigmabanModel::~SigmabanModel()
{
}
        
SigmabanModel::SupportFoot SigmabanModel::getSupportFoot() const
{
    return _supportFoot;
}
        
std::string SigmabanModel::supportFootName() const
{
    if (_supportFoot == LeftSupportFoot) {
        return "left foot tip";
    } else if (_supportFoot == RightSupportFoot) {
        return "right foot tip";
    } else {
        return "";
    }
}
std::string SigmabanModel::movingFootName() const
{
    if (_supportFoot == LeftSupportFoot) {
        return "right foot tip";
    } else if (_supportFoot == RightSupportFoot) {
        return "left foot tip";
    } else {
        return "";
    }
}

void SigmabanModel::putOnGround()
{
    putSupportFootFlat();
    putSupportFootOrigin();
    findSupportFoot();
}
        
void SigmabanModel::boundingBox(size_t frameIndex, 
    double& sizeX, double& sizeY, double& sizeZ,
    Eigen::Vector3d& center) const
{
    if (Model::getFrameName(frameIndex) == "left foot tip") {
        sizeX = 0.062495;
        sizeY = 0.039995;
        sizeZ = 0.01;
        center = Eigen::Vector3d(0.001845, 0.002495, 0.01);
    } else if (Model::getFrameName(frameIndex) == "right foot tip") {
        sizeX = 0.062495;
        sizeY = 0.039995;
        sizeZ = 0.01;
        center = Eigen::Vector3d(0.001845, -0.002495, 0.01);
    } else {
        Model::boundingBox(frameIndex, 
            sizeX, sizeY, sizeZ, center);
    }
}
        
void SigmabanModel::findSupportFoot()
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
        
void SigmabanModel::putSupportFootFlat()
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
    Model::setDOF("trunk roll", angles(0));
    Model::setDOF("trunk pitch", angles(1));
    Model::setDOF("trunk yaw", angles(2));
}
        
void SigmabanModel::putSupportFootOrigin()
{
    //Support foot position in trunk frame
    Eigen::Vector3d posFoot = Model::position(supportFootName(), "trunk");
    //Convertion of TF vector (T := trunk base origin, F := support foot tip)
    //from trunk frame to world frame
    Eigen::Matrix3d rotation = Model::orientation("origin", "trunk");
    posFoot = rotation*posFoot;
    //Apply translations
    Model::setDOF("trunk Tx", -posFoot.x() + _statePosX);
    Model::setDOF("trunk Ty", -posFoot.y() + _statePosY);
    Model::setDOF("trunk Tz", -posFoot.z());
}

}

