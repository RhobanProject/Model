#ifndef LEPH_SDKINTERFACE_HPP
#define LEPH_SDKINTERFACE_HPP

#include "Ncurses/InterfaceCLI.hpp"
#include "SDKConnection.hpp"

namespace Leph {

/**
 * SDKInterface
 *
 * Extends Ncurses CLI Interface with
 * control for the robot using the SDK
 */
class SDKInterface : public InterfaceCLI
{
    public:

        /**
         * Initialization with SDK connection instance
         */
        SDKInterface(Leph::SDKConnection& connection,
            const std::string& title);

    private:

        /**
         * SDKConnection instance reference
         */
        Leph::SDKConnection& _connection;
};

}

#endif

