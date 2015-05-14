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
         * (Yaw-Pitch-Roll) with respect to given frame name.
         * "foot tip init" is a special frame name representing a frame 
         * bound to trunk frame with initial (zero angles) 
         * foot tip translation.
         * True is returned if angles are updated and inverse
         * kinematics is sucessful, else false is returned.
         */
        bool legIkLeft(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Vector3d& yawPitchRoll = Eigen::Vector3d::Zero());
        bool legIkRight(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Vector3d& yawPitchRoll = Eigen::Vector3d::Zero());

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
         * Convert YawPitchRoll euler angle to
         * rotation matrix
         */
        Eigen::Matrix3d eulersToMatrix
            (const Eigen::Vector3d angles) const;

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
            const Eigen::Vector3d& yawPitchRoll);

        /**
         * Assign model leg DOF to given IK results
         */
        void setIKResult(
            const LegIK::Position& result, bool isLeftLeg);
};

}

#endif

