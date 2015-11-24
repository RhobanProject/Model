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
 * All combinations
 * of Euler angles types
 * in same order as rotation application
 *
 * EulerYawPitchRoll is built as
 * Roll * Pitch * Yaw.
 */
enum EulerType {
    EulerYawPitchRoll,
    EulerYawRollPitch,
    EulerRollPitchYaw,
    EulerRollYawPitch,
    EulerPitchRollYaw,
    EulerPitchYawRoll,
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
         * and enable floating base 6 DOF
         */
        HumanoidModel(
            RobotType type,
            const std::string& frameRoot);
        
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
         * foot tip at given position and euler angles orientation
         * (default is Yaw-Pitch-Roll) with respect to given frame name.
         * Used Eulers angle convention is given by eulerType.
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
            const Eigen::Vector3d& angles = Eigen::Vector3d::Zero(),
            EulerType eulerType = EulerYawPitchRoll);
        bool legIkRight(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Vector3d& angles = Eigen::Vector3d::Zero(),
            EulerType eulerType = EulerYawPitchRoll);

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
         * Compute 3d position in world frame of
         * given normalized 2d pixel coordinate
         * projected on the ground.
         * params is used camera parameters.
         * pixel is normalized between -1 and 1 relatively
         * to image width and height in screen frame (X, Y).
         * pos is updated position on the ground in
         * world frame.
         * False is returned if asked point is above
         * the horizon and pos is not updated.
         */
        bool cameraPixelToWorld(
            const CameraParameters& params,
            const Eigen::Vector2d& pixel,
            Eigen::Vector3d& pos);

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
         * Convert given euler angle of given
         * convention type to rotation matrix
         */
        Eigen::Matrix3d eulersToMatrix(
            const Eigen::Vector3d angles, EulerType eulerType) const;

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
            const Eigen::Vector3d& angles, 
            EulerType eulerType);

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

