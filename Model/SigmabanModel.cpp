#include "Model/RBDLRootUpdate.h"
#include "Model/SigmabanModel.hpp"
#include "LegIK/LegIK.hpp"

namespace Leph {

SigmabanModel::SigmabanModel(const std::string& frameRoot) :
    Model()
{
    //Load model from URDF file
    RBDL::Model modelOld;
    if (!RBDL::Addons::URDFReadFromFile(
        "sigmaban.urdf", &modelOld, false)
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
    _trunkToHipLeft = Model::position("left hip roll", "trunk");
    _trunkToHipRight = Model::position("right hip roll", "trunk");
}
        
SigmabanModel::~SigmabanModel()
{
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

bool SigmabanModel::legIkLeft(const std::string& frame,
    const Eigen::Vector3d& footPos, 
    const Eigen::Vector3d& yawPitchRoll)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, 
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    Eigen::Vector3d target = Model::position(
        frame, "trunk", footPos);
    target -= _trunkToHipLeft; 
    //Convert orientation from given frame
    //to LegIK base
    Eigen::Matrix3d rotMatrixFrame = eulersToMatrix(yawPitchRoll);
    Eigen::Matrix3d rotMatrixtarget = 
        rotMatrixFrame * Model::orientation(frame, "trunk");
    
    //Building LegIK input target position 
    //and orientation data structure
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = target(0);
    legIKTarget[1] = target(1);
    legIKTarget[2] = target(2);
    LegIK::Frame3D legIKMatrix;
    legIKMatrix[0][0] = rotMatrixtarget(0, 0);
    legIKMatrix[0][1] = rotMatrixtarget(0, 1);
    legIKMatrix[0][2] = rotMatrixtarget(0, 2);
    legIKMatrix[1][0] = rotMatrixtarget(1, 0);
    legIKMatrix[1][1] = rotMatrixtarget(1, 1);
    legIKMatrix[1][2] = rotMatrixtarget(1, 2);
    legIKMatrix[2][0] = rotMatrixtarget(2, 0);
    legIKMatrix[2][1] = rotMatrixtarget(2, 1);
    legIKMatrix[2][2] = rotMatrixtarget(2, 2);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result);

    //Update degrees of freeodm on success
    if (isSucess) {
        Model::setDOF("left hip yaw", result.theta[0]);
        Model::setDOF("left hip roll", result.theta[1]);
        Model::setDOF("left hip pitch", -result.theta[2]);
        Model::setDOF("left knee", result.theta[3]);
        Model::setDOF("left foot pitch", -result.theta[4]);
        Model::setDOF("left foot roll", result.theta[5]);
    } 

    return isSucess;
}
bool SigmabanModel::legIkRight(const std::string& frame,
    const Eigen::Vector3d& footPos, 
    const Eigen::Vector3d& yawPitchRoll)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, 
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    Eigen::Vector3d target = Model::position(
        frame, "trunk", footPos);
    target -= _trunkToHipRight; 
    //Convert orientation from given frame
    //to LegIK base
    Eigen::Matrix3d rotMatrixFrame = eulersToMatrix(yawPitchRoll);
    Eigen::Matrix3d rotMatrixtarget = 
        rotMatrixFrame * Model::orientation(frame, "trunk");
    
    //Building LegIK input target position 
    //and orientation data structure
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = target(0);
    legIKTarget[1] = target(1);
    legIKTarget[2] = target(2);
    LegIK::Frame3D legIKMatrix;
    legIKMatrix[0][0] = rotMatrixtarget(0, 0);
    legIKMatrix[0][1] = rotMatrixtarget(0, 1);
    legIKMatrix[0][2] = rotMatrixtarget(0, 2);
    legIKMatrix[1][0] = rotMatrixtarget(1, 0);
    legIKMatrix[1][1] = rotMatrixtarget(1, 1);
    legIKMatrix[1][2] = rotMatrixtarget(1, 2);
    legIKMatrix[2][0] = rotMatrixtarget(2, 0);
    legIKMatrix[2][1] = rotMatrixtarget(2, 1);
    legIKMatrix[2][2] = rotMatrixtarget(2, 2);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result);

    //Update degrees of freeodm on success
    if (isSucess) {
        Model::setDOF("right hip yaw", result.theta[0]);
        Model::setDOF("right hip roll", result.theta[1]);
        Model::setDOF("right hip pitch", -result.theta[2]);
        Model::setDOF("right knee", result.theta[3]);
        Model::setDOF("right foot pitch", -result.theta[4]);
        Model::setDOF("right foot roll", result.theta[5]);
    } 

    return isSucess;
}
        
Eigen::Matrix3d SigmabanModel::eulersToMatrix
    (const Eigen::Vector3d angles) const
{
    Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> quat = rollRot * pitchRot * yawRot;
    return quat.matrix();
}

}

