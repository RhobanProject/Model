#ifndef LEPH_OPTIMIZABLE_HPP
#define LEPH_OPTIMIZABLE_HPP

#include <iostream>
#include <stdexcept>
#include <vector>
#include "TimeSeries/MetaParameter.hpp"

namespace Leph {

/**
 * Optimizable
 *
 * Base interface for class
 * containing optimizable meta
 * parameters.
 * resetParameters() have to be called to
 * initialize the parameters container.
 */
class Optimizable
{
    public:

        /**
         * Initialization
         */
        Optimizable() :
            _parameters()
        {
        }

        /**
         * Virtual destructor
         */
        virtual ~Optimizable()
        {
        }

        /**
         * Recreate parameters container with
         * default values
         */
        inline void resetParameters()
        {
            //Clear parameters
            _parameters.clear();
            //Initialize the parameters 
            //container with default values
            size_t size = parameterSize();
            for (size_t i=0;i<size;i++) {
                _parameters.push_back(defaultParameter(i));
            }
        }
        
        /**
         * Read access to meta parameter
         * at given index
         */
        inline const MetaParameter& getParameter(size_t index) const
        {
            size_t paramSize = parameterSize();
            if (paramSize != _parameters.size()) {
                throw std::logic_error("Optimizable parameters not resetted");
            }
            if (index >= paramSize) {
                throw std::logic_error(
                    "Optimizable invalid parameter index");
            }

            return _parameters[index];
        }

        /**
         * Update the internal meta parameter with
         * given index and given value.
         * False is returned if given value is trimmed to
         * fit parameter bounds
         */
        inline bool setParameter(size_t index, double value)
        {
            size_t paramSize = parameterSize();
            if (paramSize != _parameters.size()) {
                resetParameters();
            }
            if (index >= paramSize) {
                throw std::logic_error(
                    "Optimizable invalid parameter index");
            }

            return _parameters[index].setValue(value);
        }

        /**
         * Return the number of used 
         * meta parameters
         * (Must be implemented)
         */
        virtual size_t parameterSize() const = 0;

        /**
         * Return the default initialization 
         * (name, min, max, value) of given meta
         * parameter index.
         * (Must be implemented)
         */
        virtual MetaParameter defaultParameter(size_t index) const = 0;

        /**
         * Display on given stream parameters 
         */
        inline void parameterPrint(std::ostream& os = std::cout)
        {
            for (size_t i=0;i<_parameters.size();i++) {
                os << "[" << i << "] " << _parameters[i] << std::endl;
            }
        }

    private:

        /**
         * Meta parameters container 
         * of size parameterSize()
         */
        std::vector<MetaParameter> _parameters;
};

}

#endif

