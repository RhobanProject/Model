#ifndef LEPH_HUMANOIDFIXEDMODEL_HPP
#define LEPH_HUMANOIDFIXEDMODEL_HPP

#include "Model/HumanoidModel.hpp"

namespace Leph {

/**
 * HumanoidFixedModel
 *
 * Contains two HumanoidModel with
 * root located at each leg tip and
 * switching between the two according to
 * supporting leg.
 */
class HumanoidFixedModel
{
    public:
        
        /**
         * Enum for support foot
         */
        enum SupportFoot {
            LeftSupportFoot,
            RightSupportFoot,
        };

        /**
         * Initialization with given model type
         */
        HumanoidFixedModel(RobotType type);
        
        /**
         * Virtual destructor
         */
        virtual ~HumanoidFixedModel();
        
        /**
         * Return the current support foot
         */
        SupportFoot getSupportFoot() const;

        /**
         * Update the current support to given 
         * support foot and update odometry
         */
        void setSupportFoot(SupportFoot foot);

        /**
         * Return Leph::HumanoidModel fixed
         * on current supporting foot tip
         */
        const HumanoidModel& get() const;
        HumanoidModel& get();

        /**
         * Update current support foot
         * and compute floating base tranformation
         * to integrate model displacement
         */
        virtual void updateBase();

        /**
         * Udpate support foot floating base (pitch, roll)
         * orientation in order that trunk
         * orientation matches given euler angle (IMU)
         */
        void setOrientation(double trunkPitch, double trunkRoll);

        /**
         * Compute and return the Zero Moment Point
         * (ZMP) in given frame. Given degrees of freedom
         * velocity and acceleration are used (to compute
         * inverse dynamics)
         */
        Eigen::Vector3d zeroMomentPoint(
            const std::string& frame,
            const Eigen::VectorXd& velocity,
            const Eigen::VectorXd& acceleration);
        Eigen::Vector3d zeroMomentPoint(
            const std::string& frame,
            const VectorLabel& velocity,
            const VectorLabel& acceleration);

    private:
        
        /**
         * Current support foot
         */
        SupportFoot _supportFoot;

        /**
         * HumanoidModel for left and right 
         * support foot
         */
        HumanoidModel _modelLeft;
        HumanoidModel _modelRight;

        /**
         * Convert given base pitch/roll torque into
         * X/Y torque in world frame
         */
        void convertFootMoment(
            double torquePitch, double torqueRoll,
            double& Mx, double& My);

        /**
         * Compute ZMP position given X/Y/Z force
         * and X/Y moment in world frame
         */
        Eigen::Vector3d computeZMP(
            double Fx, double Fy, double Fz, 
            double Mx, double My);
};

}

#endif

