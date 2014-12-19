#ifndef LEPH_CARTWALKPROXY_HPP
#define LEPH_CARTWALKPROXY_HPP

#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalk.h"

namespace Leph {

/**
 * CartWalkProxy
 *
 * Clean API for Rhoban
 * CartWalk motor primitive
 */
class CartWalkProxy
{
    public:

        /**
         * Initialization
         */
        CartWalkProxy();

        /**
         * Return the cyclic movement
         * phase between 0 and 1
         */
        double getPhase() const;

        /**
         * Set movement phase
         * (between 0 and 1)
         */
        void setPhase(double phase);

        /**
         * Return initialized label vector
         * for motor outputs, static parameters,
         * and dynamic parameters with default
         * values
         */
        VectorLabel buildOutputs() const;
        VectorLabel buildStaticParams() const;
        VectorLabel buildDynamicParams() const;

        /**
         * Return a labeled vector of motor outputs
         */
        VectorLabel exec(
            double deltaTime,
            const VectorLabel& dynamicParams, 
            const VectorLabel& staticParams);

    private:

        Rhoban::CartWalk _walk;
};

}

#endif

