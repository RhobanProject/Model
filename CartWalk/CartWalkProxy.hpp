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
         * for motor outputs, walk information, 
         * static parameters, and dynamic parameters 
         * with default values
         */
        VectorLabel buildOutputs() const;
        VectorLabel buildInfo() const;
        VectorLabel buildStaticParams() const;
        VectorLabel buildDynamicParams() const;

        /**
         * Return label vector with minimum
         * and maximum bound for static and
         * dynamic parameters
         */
        VectorLabel buildStaticParamsMin() const;
        VectorLabel buildStaticParamsMax() const;
        VectorLabel buildDynamicParamsMin() const;
        VectorLabel buildDynamicParamsMax() const;
        
        /**
         * Return VectorLabel with typical delta
         * length scale for all static parameters
         * (1/50 of max-min range)
         */
        VectorLabel buildStaticParamsDelta() const;

        /**
         * Return a labeled vector of motor outputs
         * given time step (updating the phase) and
         * walk parameters
         */
        VectorLabel exec(
            double deltaTime,
            const VectorLabel& dynamicParams, 
            const VectorLabel& staticParams);

        /**
         * Return last computed VectorLabel
         * outputs and info (walk information)
         */
        VectorLabel lastOutputs() const;
        VectorLabel lastInfo() const;

    private:

        /**
         * Wrapped CartWalk
         */
        Rhoban::CartWalk _walk;

        /**
         * Last computed Outputs
         * and Info
         */
        VectorLabel _lastOutputs;
        VectorLabel _lastInfo;
};

}

#endif

