#ifndef LEPH_SDKCONNECTION_HPP
#define LEPH_SDKCONNECTION_HPP

//SDK
#include "main/Command.h"
#include "robot/Robot.h"
#include "robot/Robots.h"

//CODE
#include "Types/VectorLabel.hpp"

namespace Leph {

/**
 * SDKConnection
 *
 * Provide utilities methods
 * to access SDK motors
 */
class SDKConnection
{
    public:

        /**
         * Initialize the connection
         * with RhobanServer throught SDK
         * (Local robot)
         */
        SDKConnection();

        /**
         * Stop the connection and dispatcher
         */
        ~SDKConnection();

        /**
         * Send given reference relative
         * angles to motors
         */
        void setMotorAngles(const Leph::VectorLabel& outputs);

        /**
         * Retrieve current motors 
         * relative angles
         */
        Leph::VectorLabel getMotorAngles();

        /**
         * Forward to SDK init and compliant 
         * (emergency) request
         */
        void init();
        void compliant();

    private:

        /**
         * SDK robot instance
         */
        Rhoban::Robot _robot;
};

}

#endif

