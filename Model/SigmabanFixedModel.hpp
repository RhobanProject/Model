#ifndef LEPH_SIGMABANFIXEDMODEL_HPP
#define LEPH_SIGMABANFIXEDMODEL_HPP

#include "Model/SigmabanModel.hpp"

namespace Leph {

/**
 * SigmabanFixedModel
 *
 * Contains two SigmabanModel with
 * root located at each leg tip and
 * switching between the two according to
 * supporting leg.
 */
class SigmabanFixedModel
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
         * Initialization with sigmaban URDF model
         */
        SigmabanFixedModel();
        
        /**
         * Return the current support foot
         */
        SupportFoot getSupportFoot() const;

        /**
         * Force given support foot
         */
        void setSupportFoot(SupportFoot foot);

        /**
         * Return Leph::SigmabanModel fixed
         * on current supporting foot tip
         */
        const SigmabanModel& get() const;
        SigmabanModel& get();

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
         * SigmabanModel for left and right 
         * support foot
         */
        SigmabanModel _modelLeft;
        SigmabanModel _modelRight;
};

}

#endif

