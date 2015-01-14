#include "SDKInterface.hpp"

namespace Leph {

SDKInterface::SDKInterface(Leph::SDKConnection& connection,
    const std::string& title) :
    InterfaceCLI(title),
    _connection(connection)
{
}

}

