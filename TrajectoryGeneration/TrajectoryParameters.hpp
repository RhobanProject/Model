#ifndef LEPH_TRAJECTORYPARAMETERS_HPP
#define LEPH_TRAJECTORYPARAMETERS_HPP

#include <string>
#include <map>
#include <iomanip>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>

namespace Leph {

/**
 * TrajectoryParameters
 *
 * Simple header container for mapping
 * name to parameters for CMA-ES optimization
 * and especially trajectory generation
 */
class TrajectoryParameters
{
    public:

        /**
         * Initialization
         */
        TrajectoryParameters() :
            _countOptimized(0),
            _container()
        {
        }

        /**
         * Define the new given name as parameter
         * with given value and flag isOptimized
         */
        inline void add(
            const std::string& name, 
            double value = 0.0, 
            bool isOptimized = false)
        {
            //Check parameter exists
            if (_container.count(name) != 0) {
                throw std::logic_error(
                    "TrajectoryParameters exist parameter: " 
                    + name);
            }
            //Create parameters
            _container[name] = Parameter();
            _container.at(name).name = name;
            _container.at(name).isOptimized = isOptimized;
            _container.at(name).defaultValue = value;
            if (isOptimized) {
                _container.at(name).index = _countOptimized;
                _countOptimized++;
            } else {
                _container.at(name).index = (size_t)-1;
            }
        }

        /**
         * Update the default value of given name
         */
        inline void set(
            const std::string& name,
            double value)
        {
            //Check parameter exists
            if (_container.count(name) == 0) {
                throw std::logic_error(
                    "TrajectoryParameters unknown parameter: " 
                    + name);
            }
            //Assign new value
            _container.at(name).defaultValue = value;
        }

        /**
         * Build and return the vector of 
         * optimized parameters
         */
        inline Eigen::VectorXd buildVector() const
        {
            //Build and assign the vector
            Eigen::VectorXd params = 
                Eigen::VectorXd::Zero(_countOptimized);
            for (const auto& it : _container) {
                if (it.second.isOptimized) {
                    params(it.second.index) = it.second.defaultValue;
                }
            }

            return params;
        }

        /**
         * Retrieve the given parameter name either
         * from unoptimized value of from given
         * parameters vector
         */
        inline double get(
            const std::string& name, 
            const Eigen::VectorXd& parameters) const
        {
            //Check parameter exists
            if (_container.count(name) == 0) {
                throw std::logic_error(
                    "TrajectoryParameters unknown parameter: " 
                    + name);
            }
            //Get from container or vector
            const Parameter& p = _container.at(name);
            if (p.isOptimized) {
                return parameters(p.index);
            } else {
                return p.defaultValue;
            }
        }
        
        /**
         * Retrieve the given vector name if exists 
         * either from unoptimized value of from given
         * parameters vector.
         * Convention is _x, _y, _z.
         */
        inline Eigen::Vector3d getVect(
            const std::string& name, 
            const Eigen::VectorXd& parameters) const
        {
            //Build name
            std::string nameX = name + "_x";
            std::string nameY = name + "_y";
            std::string nameZ = name + "_z";
            //Check of elements exist
            if (
                _container.count(nameX) == 0 ||
                _container.count(nameY) == 0 ||
                _container.count(nameZ) == 0
            ) {
                throw std::logic_error(
                    "TrajectoryParameters unknown vector: " 
                    + name);
            }
            //Retrieve from value or parameters vector
            const Parameter& pX = _container.at(nameX);
            const Parameter& pY = _container.at(nameY);
            const Parameter& pZ = _container.at(nameZ);
            Eigen::Vector3d vect(0.0, 0.0, 0.0);
            //X
            if (pX.isOptimized) {
                vect.x() = parameters(pX.index);
            } else {
                vect.x() = pX.defaultValue;
            }
            //Y
            if (pY.isOptimized) {
                vect.y() = parameters(pY.index);
            } else {
                vect.y() = pY.defaultValue;
            }
            //Z
            if (pZ.isOptimized) {
                vect.z() = parameters(pZ.index);
            } else {
                vect.z() = pZ.defaultValue;
            }

            return vect;
        }

        /**
         * Print with given output stream the 
         * current container state
         */
        inline void print(
            std::ostream& os = std::cout) const
        {
            for (const auto& it : _container) {
                os << "[" << std::setw(20) << it.first << ":";
                if (it.second.isOptimized) {
                    os << std::setw(3) << it.second.index << "] ";
                } else {
                    os << std::setw(5) << "] ";
                }
                os << it.second.defaultValue;
                os << std::endl;
            }
        }

    private:

        /**
         * Parameter structure
         */
        struct Parameter {
            std::string name;
            bool isOptimized;
            double defaultValue;
            size_t index;
        };

        /**
         * Number of contained 
         * optimized parameters
         */
        size_t _countOptimized;

        /**
         * Mapping name to parameters
         */
        std::map<std::string, Parameter> _container;
};

}

#endif

