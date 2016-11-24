#ifndef LEPH_DEFAULTTRAJPARAMETERS_H
#define LEPH_DEFAULTTRAJPARAMETERS_H

#include "TrajectoryGeneration/TrajectoryParameters.hpp"

namespace Leph {

/**
 * Return default TrajectoryParameters instance.
 * Definition of CMA-ES and static single 
 * and double support parameters.
 */
TrajectoryParameters DefaultTrajParameters();

}

#endif

