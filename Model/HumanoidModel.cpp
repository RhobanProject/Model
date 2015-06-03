#include "Model/RBDLRootUpdate.h"
#include "Model/HumanoidModel.hpp"

namespace Leph {

HumanoidModel::HumanoidModel(
    RobotType type,
    const std::string& frameRoot) :
    Model(),
    _type(type)
{
    //Select used URDF model file
    std::string urdfFile;
    if (_type == SigmabanModel) {
        urdfFile = "sigmaban.urdf";
    } else if (_type == GrosbanModel) {
        urdfFile = "grosban.urdf";
    }

    //Load model from URDF file
    RBDL::Model modelOld;
    if (!RBDL::Addons::URDFReadFromFile(
        urdfFile.c_str(), &modelOld, false)
    ) {
        std::runtime_error("Model unable to load URDF file");
    }

    //Select new RBDL body id root
    size_t frameRootId;
    if (frameRoot == "ROOT") {
        frameRootId = 0;
    } else {
        Leph::Model wrappedModelNew(modelOld);
        frameRootId = wrappedModelNew.frameIndexToBodyId(
            wrappedModelNew.getFrameIndex(frameRoot));
    }

    //Update old urdf model with new root frame
    RBDL::Model modelNew = 
        Leph::RBDLRootUpdate(modelOld, frameRootId, true);
    //Initialize base model
    Model::initilializeModel(modelNew);

    //Compute leg segments length
    Eigen::Vector3d hipPt = Model::position(
        "right hip roll", "origin");
    Eigen::Vector3d kneePt = Model::position(
        "right knee", "origin");
    Eigen::Vector3d anklePt = Model::position(
        "right foot pitch", "origin");
    Eigen::Vector3d footPt = Model::position(
        "right foot tip", "origin");
    _legHipToKnee = (hipPt-kneePt).norm();
    _legKneeToAnkle = (kneePt-anklePt).norm();
    _legAnkleToGround = (anklePt-footPt).norm();
    //Compute standart translation 
    //from trunk in zero position
    _trunkToHipLeft = Model::position("left hip roll", "trunk");
    _trunkToHipRight = Model::position("right hip roll", "trunk");
    _trunkToFootTipLeft = Model::position("left foot tip", "trunk");
    _trunkToFootTipRight = Model::position("right foot tip", "trunk");
}
        
HumanoidModel::~HumanoidModel()
{
}
        
void HumanoidModel::boundingBox(size_t frameIndex, 
    double& sizeX, double& sizeY, double& sizeZ,
    Eigen::Vector3d& center) const
{
    if (_type == SigmabanModel) {
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
    } else if (_type == GrosbanModel) {
        if (Model::getFrameName(frameIndex) == "left foot tip") {
            sizeX = 0.1225;
            sizeY = 0.065;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.0034, 0.036, 0.01);
        } else if (Model::getFrameName(frameIndex) == "right foot tip") {
            sizeX = 0.1225;
            sizeY = 0.065;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.0034, -0.036, 0.01);
        } else {
            Model::boundingBox(frameIndex, 
                sizeX, sizeY, sizeZ, center);
        }
    }
}

bool HumanoidModel::legIkLeft(const std::string& frame,
    const Eigen::Vector3d& footPos, 
    const Eigen::Vector3d& angles,
    EulerType eulerType)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, 
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    LegIK::Vector3D legIKTarget = buildTargetPos(
        frame, footPos, true);
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix = buildTargetOrientation(
        frame, angles, eulerType);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result);

    //Update degrees of freedom on success
    if (isSucess) {
        checkNaN(result, legIKTarget, legIKMatrix);
        setIKResult(result, true);
    } 

    return isSucess;
}
bool HumanoidModel::legIkRight(const std::string& frame,
    const Eigen::Vector3d& footPos, 
    const Eigen::Vector3d& angles,
    EulerType eulerType)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, 
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    LegIK::Vector3D legIKTarget = buildTargetPos(
        frame, footPos, false);
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix = buildTargetOrientation(
        frame, angles, eulerType);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result);

    //Update degrees of freedom on success
    if (isSucess) {
        checkNaN(result, legIKTarget, legIKMatrix);
        setIKResult(result, false);
    } 

    return isSucess;
}
        
double HumanoidModel::legsLength() const
{
    return -_trunkToFootTipLeft.z();
}
        
double HumanoidModel::feetDistance() const
{
    return _trunkToHipLeft.y() - _trunkToHipRight.y();
}
        
Eigen::Matrix3d HumanoidModel::eulersToMatrix(
    const Eigen::Vector3d angles, EulerType eulerType) const
{
    Eigen::Quaternion<double> quat;
    switch (eulerType) {
        case EulerYawPitchRoll: {
            Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
            quat = rollRot * pitchRot * yawRot;
        }
        break;
        case EulerYawRollPitch: {
            Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
            quat = pitchRot * rollRot * yawRot;
        }
        break;
        case EulerRollPitchYaw: {
            Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
            quat = yawRot * pitchRot * rollRot;
        }
        break;
        case EulerRollYawPitch: {
            Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
            quat = pitchRot * yawRot * rollRot;
        }
        break;
        case EulerPitchRollYaw: {
            Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
            quat = yawRot * rollRot * pitchRot;
        }
        break;
        case EulerPitchYawRoll: {
            Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
            quat = rollRot * yawRot * pitchRot;
        }
        break;
    }
    return quat.matrix();
}

LegIK::Vector3D HumanoidModel::buildTargetPos(
    const std::string& frame,
    const Eigen::Vector3d& footPos, 
    bool isLeftLeg)
{
    Eigen::Vector3d target;
    if (frame == "foot tip init") {
        //Special frame where foot tip in zero position
        target = footPos;
        if (isLeftLeg) {
            target += _trunkToFootTipLeft;
            target -= _trunkToHipLeft; 
        } else {
            target += _trunkToFootTipRight;
            target -= _trunkToHipRight; 
        }
    } else {
        target = Model::position(
            frame, "trunk", footPos);
        if (isLeftLeg) {
            target -= _trunkToHipLeft; 
        } else {
            target -= _trunkToHipRight; 
        }
    }

    //Building LegIK input target position 
    //data structure
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = target(0);
    legIKTarget[1] = target(1);
    legIKTarget[2] = target(2);
    return legIKTarget;
}
LegIK::Frame3D HumanoidModel::buildTargetOrientation(
    const std::string& frame,
    const Eigen::Vector3d& angles, 
    EulerType eulerType)
{
    Eigen::Matrix3d rotMatrixFrame = eulersToMatrix(
        angles, eulerType);
    Eigen::Matrix3d rotMatrixTarget = rotMatrixFrame;
    if (frame == "foot tip init") {
        //Special frame where foot tip in zero position
        //No conversion
    } else {
        rotMatrixFrame *= Model::orientation(frame, "trunk");
    }

    //Building LegIK input target
    //orientation data structure
    LegIK::Frame3D legIKMatrix;
    legIKMatrix[0][0] = rotMatrixTarget(0, 0);
    legIKMatrix[0][1] = rotMatrixTarget(0, 1);
    legIKMatrix[0][2] = rotMatrixTarget(0, 2);
    legIKMatrix[1][0] = rotMatrixTarget(1, 0);
    legIKMatrix[1][1] = rotMatrixTarget(1, 1);
    legIKMatrix[1][2] = rotMatrixTarget(1, 2);
    legIKMatrix[2][0] = rotMatrixTarget(2, 0);
    legIKMatrix[2][1] = rotMatrixTarget(2, 1);
    legIKMatrix[2][2] = rotMatrixTarget(2, 2);
    return legIKMatrix;
}
        
void HumanoidModel::setIKResult(
    const LegIK::Position& result, bool isLeftLeg)
{
    std::string prefix;
    if (isLeftLeg) {
        prefix = "left ";
    } else {
        prefix = "right ";
    }

    Model::setDOF(prefix+"hip yaw", result.theta[0]);
    Model::setDOF(prefix+"hip roll", result.theta[1]);
    if (_type == GrosbanModel) {
        //Handle non alignement in zero position
        //of hip and ankle Z (zaw) axes (knee angle of Grosban)
        Model::setDOF(prefix+"hip pitch", -result.theta[2] - 0.0603);
        Model::setDOF(prefix+"knee", result.theta[3] + 0.0603);
        Model::setDOF(prefix+"foot pitch", -result.theta[4] + 0.0035);
    } else {
        Model::setDOF(prefix+"hip pitch", -result.theta[2]);
        Model::setDOF(prefix+"knee", result.theta[3]);
        Model::setDOF(prefix+"foot pitch", -result.theta[4]);
    }
    Model::setDOF(prefix+"foot roll", result.theta[5]);
}

void HumanoidModel::checkNaN(
    const LegIK::Position& result, 
    const LegIK::Vector3D& pos,
    const LegIK::Frame3D& orientation) const
{
    //Check if Nan is returned
    if (
        std::isnan(result.theta[0]) ||
        std::isnan(result.theta[1]) ||
        std::isnan(result.theta[2]) ||
        std::isnan(result.theta[3]) ||
        std::isnan(result.theta[4]) ||
        std::isnan(result.theta[5])
    ) {
        throw std::logic_error("LegIK NaN invalid result. "
            + std::string("theta0=") 
            + std::to_string(result.theta[0]) 
            + std::string(" ")
            + std::string("theta1=") 
            + std::to_string(result.theta[1]) 
            + std::string(" ")
            + std::string("theta2=") 
            + std::to_string(result.theta[2]) 
            + std::string(" ")
            + std::string("theta3=") 
            + std::to_string(result.theta[3]) 
            + std::string(" ")
            + std::string("theta4=") 
            + std::to_string(result.theta[4]) 
            + std::string(" ")
            + std::string("theta5=") 
            + std::to_string(result.theta[5]) 
            + std::string(" pos=")
            + pos.pp()
            + std::string(" orientation=")
            + orientation.pp()
        );
    }

}

}

