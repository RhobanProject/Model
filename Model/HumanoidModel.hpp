#ifndef LEPH_HUMANOIDMODEL_HPP
#define LEPH_HUMANOIDMODEL_HPP

#include "Model/Model.hpp"

namespace Leph {

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
         * URDF file and root updater
         * and enable floating base 6 DOF
         */
        HumanoidModel(
            const std::string& urdfFile,
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
         * True is returned if angles are updated and inverse
         * kinematics is sucessful, else false is returned.
         */
        bool legIkLeft(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Vector3d& yawPitchRoll = Eigen::Vector3d::Zero());
        bool legIkRight(const std::string& frame,
            const Eigen::Vector3d& footPos, 
            const Eigen::Vector3d& yawPitchRoll = Eigen::Vector3d::Zero());

    private:

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
         */
        Eigen::Vector3d _trunkToHipLeft;
        Eigen::Vector3d _trunkToHipRight;

        /**
         * Convert YawPitchRoll euler angle to
         * rotation matrix
         */
        Eigen::Matrix3d eulersToMatrix
            (const Eigen::Vector3d angles) const;
};

}

#endif

