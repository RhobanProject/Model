#ifndef LEPH_HUMANOIDMODEL_HPP
#define LEPH_HUMANOIDMODEL_HPP

#include "Model/Model.hpp"
#include "LegIK/LegIK.hpp"

namespace Leph {

/**
 * Enum for humanoid robot
 * model type 
 * (Sigmaban or Grosban)
 */
enum RobotType {
    SigmabanModel,
    GrosbanModel,
};

/**
 * Camera pixel 2D coordinates to
 * 3D world coordinates parameters
 */
struct CameraParameters {
    //Width and height angular aperture in radians
    double widthAperture;
    double heightAperture;
};

/**
 * HumanoidModel
 *
 * Inherit Model and implement
 * Sigmaban and Grosban 
 * feet bounding box and inverse
 * kinematics interface
 */
class HumanoidModel : public Model
{
    public:

        /**
         * Initialize the model with given
         * Robot type and root updater
         * and enable floating base 6 DOF if
         * isFloatingBase is true
         */
        HumanoidModel(
            RobotType type,
            const std::string& frameRoot,
            bool isFloatingBase = true);
        
        /**
         * Virtual destructor
         */
        virtual ~HumanoidModel();
        
        /**
         * @Inherit
         * Draw feet bounding box
         */
        void boundingBox(size_t frameIndex, 
            double& sizeX, double& sizeY, double& sizeZ,
            Eigen::Vector3d& center) const override;

        /**
         * Run analytical inverse kinematics LegIK and update
         * Left ot Right legs angles to place the Left or Right
         * foot tip at given position and rotation matrix for orientation
         * with respect to given frame name.
         * "foot tip init" is a special frame name representing a frame 
         * bound to trunk frame with initial (zero angles) 
         * foot tip translation.
         * "LegIK" is the raw reference frame of LegIK 
         * implementation.
         * True is returned if angles are updated and inverse
         * kinematics is sucessful, else false is returned.
         */
        bool legIkLeft(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity());
        bool legIkRight(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity());

        /**
         * Return the initial vertical distance
         * from trunk frame to foot tip frame (Z)
         */
        double legsLength() const;

        /**
         * Return the initial lateral distance
         * between each feet
         */
        double feetDistance() const;

        /**
         * Return the rotation matrix and translation vector
         * expressing given frame into the robot self frame
         * (flat (pitch/roll) on ground at the vertical of trunk).
         */
        Eigen::Matrix3d selfFrameOrientation(const std::string& frame);
        Eigen::Vector3d selfFramePosition(const std::string& frame);

        /**
         * selfInFrame: return the position of the point expressed in self
         * robot frame (zero as default) into given frame name.
         * frameInSelf: return the position of the point expressed in given
         * frame name (zero as default) into robot self frame.
         */
        Eigen::Vector3d selfInFrame(
            const std::string& name, 
            const Eigen::Vector3d& pos = Eigen::Vector3d::Zero());
        Eigen::Vector3d frameInSelf(
            const std::string& name, 
            const Eigen::Vector3d& pos = Eigen::Vector3d::Zero());

        /**
         * Compute 3d position in world frame (origin) 
         * of given normalized 2d pixel coordinate
         * projected on the ground.
         * params is used camera parameters.
         * pixel is normalized between -1 and 1 relatively
         * to image width and height in screen frame (X, Y).
         * pos is updated position on the ground in
         * world frame.
         * False is returned if asked point is above
         * the horizon and pos is shrink to the horizon line.
         */
        bool cameraPixelToWorld(
            const CameraParameters& params,
            const Eigen::Vector2d& pixel,
            Eigen::Vector3d& pos);

        /**
         * Compute normalize 2d pixel position in camera
         * space projected from given point in world
         * frame (origin). 
         * Given Camera parameters are used.
         * False is returned if projection fails (not 
         * inversible) or if projected point comes 
         * from camera's backside.
         */
        bool cameraWorldToPixel(
            const CameraParameters& params,
            const Eigen::Vector3d& pos,
            Eigen::Vector2d& pixel);

        /**
         * Set head yaw and pitch degrees of
         * freedom to look at given target position
         * in world (origin) frame.
         * If offsetPixelTilt is zero, the given
         * worl point is set at the center of the
         * camera view.
         * If offsetPixelTilt is between -1 and 1 in
         * pixel space, the given world point is 
         * centered at the camera view in width 
         * but is offset in height.
         * Camera parameters is given as input.
         * In NoUpdate version, the underlying model
         * is not updated and given reference dof 
         * are assigned.
         */
        void cameraLookAt(
            const CameraParameters& params,
            const Eigen::Vector3d& posTarget,
            double offsetPixelTilt = 0.0);
        void cameraLookAtNoUpdate(
            double& panDOF,
            double& tiltDOF,
            const CameraParameters& params,
            const Eigen::Vector3d& posTarget,
            double offsetPixelTilt = 0.0);

        /**
         * Compute and return the height in
         * pixel normalized coordinate of the 
         * horizon line at given width pixel
         * normalized coordinate.
         * params is used camera parameters.
         * screenWidth is width (X) pixel 
         * coordinate between -1 and 1.
         */
        double cameraScreenHorizon(
            const CameraParameters& params,
            double screenPosWidth);

    private:

        /**
         * Robot type (Sigmaban or Grosban)
         */
        RobotType _type;

        /**
         * Leg segments lengths used by
         * inverse kinematics
         */
        double _legHipToKnee;
        double _legKneeToAnkle;
        double _legAnkleToGround;

        /**
         * Translation from trunk frame
         * to hip frame in Zero position
         * (intersection of hip yaw/pitch/roll axes)
         * and to foot tip in Zero position.
         */
        Eigen::Vector3d _trunkToHipLeft;
        Eigen::Vector3d _trunkToHipRight;
        Eigen::Vector3d _trunkToFootTipLeft;
        Eigen::Vector3d _trunkToFootTipRight;

        /**
         * Neck segment lengts used by
         * camera inverse kinematics
         */
        double _headYawToPitch;
        double _headPitchToCameraZ;
        double _headPitchToCameraX;

        /**
         * Compute and return the IK position reference
         * vector and orientation reference matrix
         * in LegIK specifics structures
         */
        LegIK::Vector3D buildTargetPos(
            const std::string& frame,
            const Eigen::Vector3d& footPos, 
            bool isLeftLeg);
        LegIK::Frame3D buildTargetOrientation(
            const std::string& frame,
            const Eigen::Matrix3d& rotation);

        /**
         * Assign model leg DOF to given IK results
         */
        void setIKResult(
            const LegIK::Position& result, bool isLeftLeg);

        /**
         * Check inverse kinematics computed value
         * and throw an error in case of NaN
         */
        void checkNaN(
            const LegIK::Position& result, 
            const LegIK::Vector3D& pos,
            const LegIK::Frame3D& orientation) const;
};

}

#endif

