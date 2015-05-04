#ifndef LEPH_SIGMABANMODEL_HPP
#define LEPH_SIGMABANMODEL_HPP

#include "Model/Model.hpp"

namespace Leph {

/**
 * SigmabanModel
 *
 * Inherit Model and implement
 * Sigmaban feet bounding box
 */
class SigmabanModel : public Model
{
    public:

        /**
         * Initialize the model with root updater
         * and enable foating base 6 DOF
         */
        SigmabanModel(const std::string& frameRoot);
        
        /**
         * Destructor
         */
        virtual ~SigmabanModel();
        
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

