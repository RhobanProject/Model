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
         * Return the current support foot
         */
        SupportFoot getSupportFoot() const;

        /**
         * Force given support foot
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
        void updateBase();

        /**
         * Udpate support foot floating base (pitch, roll)
         * orientation in order that trunk
         * orientation matches given euler angle (IMU)
         */
        void setOrientation(double trunkPitch, double trunkRoll);

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
};

}

#endif

