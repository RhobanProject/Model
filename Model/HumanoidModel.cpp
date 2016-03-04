#include "Model/RBDLRootUpdate.h"
#include "Model/HumanoidModel.hpp"

namespace Leph {

HumanoidModel::HumanoidModel(
    RobotType type,
    const std::string& frameRoot,
    bool isFloatingBase) :
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
        Leph::RBDLRootUpdate(modelOld, frameRootId, isFloatingBase);
    //Initialize base model
    Model::initializeModel(modelNew);

    //Compute leg segments length
    Eigen::Vector3d hipPt = Model::position(
        "right_hip_roll", "origin");
    Eigen::Vector3d kneePt = Model::position(
        "right_knee", "origin");
    Eigen::Vector3d anklePt = Model::position(
        "right_ankle_pitch", "origin");
    Eigen::Vector3d footPt = Model::position(
        "right_foot_tip", "origin");
    _legHipToKnee = (hipPt-kneePt).norm();
    _legKneeToAnkle = (kneePt-anklePt).norm();
    _legAnkleToGround = (anklePt-footPt).norm();
    //Compute standart translation 
    //from trunk in zero position
    _trunkToHipLeft = Model::position("left_hip_roll", "trunk");
    _trunkToHipRight = Model::position("right_hip_roll", "trunk");
    _trunkToFootTipLeft = Model::position("left_foot_tip", "trunk");
    _trunkToFootTipRight = Model::position("right_foot_tip", "trunk");
}
        
HumanoidModel::~HumanoidModel()
{
}
        
void HumanoidModel::boundingBox(size_t frameIndex, 
    double& sizeX, double& sizeY, double& sizeZ,
    Eigen::Vector3d& center) const
{
    if (_type == SigmabanModel) {
        if (Model::getFrameName(frameIndex) == "left_foot_tip") {
            sizeX = 0.062495;
            sizeY = 0.039995;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.001845, 0.002495, 0.01);
        } else if (Model::getFrameName(frameIndex) == "right_foot_tip") {
            sizeX = 0.062495;
            sizeY = 0.039995;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.001845, -0.002495, 0.01);
        } else {
            Model::boundingBox(frameIndex, 
                sizeX, sizeY, sizeZ, center);
        }
    } else if (_type == GrosbanModel) {
        if (Model::getFrameName(frameIndex) == "left_foot_tip") {
            sizeX = 0.1225;
            sizeY = 0.065;
            sizeZ = 0.01;
            center = Eigen::Vector3d(0.0034, 0.036, 0.01);
        } else if (Model::getFrameName(frameIndex) == "right_foot_tip") {
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
    const Eigen::Matrix3d& rotation)
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
        frame, rotation);
    
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
    const Eigen::Matrix3d& rotation)
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
        frame, rotation);
    
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

bool HumanoidModel::cameraPixelToWorld(
    const CameraParameters& params,
    const Eigen::Vector2d& pixel,
    Eigen::Vector3d& pos)
{
    double focalLength = 0.01;
    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");
    //Camera orientation
    Eigen::Matrix3d orientation = Model::orientation("camera", "origin");
    orientation.transposeInPlace();

    //Half width and height aperture distance on focal plane
    double widthLen = focalLength*sin(params.widthAperture);
    double heightLen = focalLength*sin(params.heightAperture);
    //Pixel width and height distance from optical center
    double pixelWidthPos = pixel.x()*widthLen;
    double pixelHeightPos = pixel.y()*heightLen;

    //Pixel position in world frame
    Eigen::Vector3d pixelPos = center
        + focalLength*orientation.col(0)
        - pixelWidthPos*orientation.col(1)
        - pixelHeightPos*orientation.col(2);

    //Unnormalize forward pixel vector 
    Eigen::Vector3d forward = pixelPos - center;

    //Check if the asked point is above the horizon
    if (forward.z() >= -0.0001) {
        return false;
    }

    //Line abscisse intersection in the ground
    double t = -center.z()/forward.z();

    //Intersection point
    Eigen::Vector3d pt = center + t*forward;

    pos = pt;
    return true;
}

double HumanoidModel::cameraScreenHorizon(
    const CameraParameters& params,
    double screenPosWidth)
{
    double focalLength = 0.01;
    //Optical center
    Eigen::Vector3d center = Model::position("camera", "origin");
    //Camera orientation
    Eigen::Matrix3d orientation = Model::orientation("camera", "origin");
    orientation.transposeInPlace();
    
    //Half width and height aperture distance on focal plane
    double widthLen = focalLength*sin(params.widthAperture);
    double heightLen = focalLength*sin(params.heightAperture);
    //Pixel width distance from optical center
    double pixelWidthPos = screenPosWidth*widthLen;
    
    //Position in world frame of asked width pixel line
    //at zero height
    Eigen::Vector3d pixelPos = center
        + focalLength*orientation.col(0)
        - pixelWidthPos*orientation.col(1);

    //Get position on vertical line where the vector
    //between the line's point and optical center is
    //horizontal
    double t = (center.z()-pixelPos.z())/orientation.col(2).z();
    //Compute this point in world frame
    Eigen::Vector3d horizonPos = pixelPos + t*orientation.col(2);

    //Conversion to optical plane vertical coordinate
    double horizonScreenHeight = 
        (horizonPos-center).dot(orientation.col(2));

    //Convertion to screen normalized height coordinate
    return -horizonScreenHeight/heightLen;
}
        
LegIK::Vector3D HumanoidModel::buildTargetPos(
    const std::string& frame,
    const Eigen::Vector3d& footPos, 
    bool isLeftLeg)
{
    Eigen::Vector3d target;
    if (frame == "foot_tip_init") {
        //Special frame where foot tip in zero position
        target = footPos;
        if (isLeftLeg) {
            target += _trunkToFootTipLeft;
            target -= _trunkToHipLeft; 
        } else {
            target += _trunkToFootTipRight;
            target -= _trunkToHipRight; 
        }
    } else if (frame == "LegIK") {
        target = footPos;
        //Raw LegIK frame
        //No transformation
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
    const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix3d rotMatrixTarget = rotation;
    if (frame == "foot_tip_init") {
        //Special frame where foot tip in zero position
        //No conversion
    } else if (frame == "LegIK") {
        //Raw LegIK frame
        //No conversion
    } else {
        rotMatrixTarget *= Model::orientation(frame, "trunk");
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
        prefix = "left_";
    } else {
        prefix = "right_";
    }

    Model::setDOF(prefix+"hip_yaw", result.theta[0]);
    Model::setDOF(prefix+"hip_roll", result.theta[1]);
    if (_type == GrosbanModel) {
        //Handle non alignement in zero position
        //of hip and ankle Z (zaw) axes (knee angle of Grosban)
        Model::setDOF(prefix+"hip_pitch", -result.theta[2] - 0.0603);
        Model::setDOF(prefix+"knee", result.theta[3] + 0.0603);
        Model::setDOF(prefix+"ankle_pitch", -result.theta[4] + 0.0035);
    } else {
        Model::setDOF(prefix+"hip_pitch", -result.theta[2]);
        Model::setDOF(prefix+"knee", result.theta[3]);
        Model::setDOF(prefix+"ankle_pitch", -result.theta[4]);
    }
    Model::setDOF(prefix+"ankle_roll", result.theta[5]);
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

