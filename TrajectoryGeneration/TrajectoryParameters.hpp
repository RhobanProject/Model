#ifndef LEPH_TRAJECTORYPARAMETERS_HPP
#define LEPH_TRAJECTORYPARAMETERS_HPP

#include <string>
#include <map>
#include <iomanip>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>
#include "TrajectoryGeneration/TrajectoryUtils.h"

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
            if (isDefined(name)) {
                throw std::logic_error(
                    "TrajectoryParameters exist parameter: " 
                    + name);
            }
            //Create parameters
            _container[name] = Parameter();
            _container.at(name).name = name;
            _container.at(name).isOptimized = isOptimized;
            _container.at(name).defaultValue = value;
            computeIndex();
        }

        /**
         * Return true is given parameter 
         * name is already contained
         */
        inline bool isDefined(
            const std::string& name) const
        {
            return (_container.count(name) != 0);
        }

        /**
         * Enable or disable a parameter optimization
         */
        inline void optimize(
            const std::string& name, bool isOptimized) 
        {
            checkName(name);
            _container.at(name).isOptimized = isOptimized;
            computeIndex();
        }

        /**
         * Return value access to 
         * given parameter name.
         * Unknown parameter are not created.
         */
        inline double& set(
            const std::string& name)
        {
            checkName(name);
            return _container.at(name).defaultValue;
        }
        
        /**
         * Return value access to 
         * given parameter name.
         * Unknown parameter are created
         * with given optimized flag.
         */
        inline double& set(
            const std::string& name,
            bool isOptimized)
        {
            if (!isDefined(name)) {
                add(name, 0.0, isOptimized);
            } else {
                _container.at(name).isOptimized = isOptimized;
                computeIndex();
            }
            return _container.at(name).defaultValue;
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
            checkName(name);
            //Get from container or vector
            const Parameter& p = _container.at(name);
            if (p.isOptimized) {
                return parameters(p.index);
            } else {
                return p.defaultValue;
            }
        }
        
        /**
         * Retrieve the default value from 
         * given parameter name 
         */
        inline double get(
            const std::string& name) const
        {
            checkName(name);
            //Get from container or vector
            const Parameter& p = _container.at(name);
            return p.defaultValue;
        }
        
        /**
         * Retrieve the given vector name if exists 
         * either from unoptimized value of from given
         * parameters vector.
         * Convention is _x, _y, _z.
         * If asked data does not exist, 
         * zero is filled.
         */
        inline Eigen::Vector3d getVect(
            const std::string& name, 
            const Eigen::VectorXd& parameters) const
        {
            //Build name
            std::string nameX = name + "_x";
            std::string nameY = name + "_y";
            std::string nameZ = name + "_z";
            
            //Build vector
            Eigen::Vector3d vect(0.0, 0.0, 0.0);
            
            //Check if elements exist
            if (isDefined(nameX)) {
                //Retrieve default value or parameter
                const Parameter& pX = _container.at(nameX);
                if (pX.isOptimized) {
                    vect.x() = parameters(pX.index);
                } else {
                    vect.x() = pX.defaultValue;
                }
            } else {
                //Zero if not exist
                vect.x() = 0.0;
            }
            
            //Check if elements exist
            if (isDefined(nameY)) {
                //Retrieve default value or parameter
                const Parameter& pY = _container.at(nameY);
                if (pY.isOptimized) {
                    vect.y() = parameters(pY.index);
                } else {
                    vect.y() = pY.defaultValue;
                }
            } else {
                //Zero if not exist
                vect.y() = 0.0;
            }
            
            //Check if elements exist
            if (isDefined(nameZ)) {
                //Retrieve default value or parameter
                const Parameter& pZ = _container.at(nameZ);
                if (pZ.isOptimized) {
                    vect.z() = parameters(pZ.index);
                } else {
                    vect.z() = pZ.defaultValue;
                }
            } else {
                //Zero if not exist
                vect.z() = 0.0;
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
                os << "[" << std::setw(40) << it.first << ":";
                if (it.second.isOptimized) {
                    os << std::setw(3) << it.second.index << "] ";
                } else {
                    os << std::setw(5) << "] ";
                }
                os << it.second.defaultValue;
                os << std::endl;
            }
        }

        /**
         * Assign to given Trajectories container
         * at given time the parameters for
         * trunk_pos, trunk_axis, foot_pos, foot_axis,
         * using following name format:
         * prefix_[pos|vel|acc]_[trunk|foot]_[pos|axis]_[x|y|z]
         * (Zero is set to missing data)
         */
        void trajectoriesAssign(
            Trajectories& traj,
            double time,
            const std::string& prefix,
            const Eigen::VectorXd& vect) const
        {
            Eigen::Vector3d posTrunkPos = 
                getVect(prefix + "_pos_trunk_pos", vect);
            Eigen::Vector3d velTrunkPos = 
                getVect(prefix + "_vel_trunk_pos", vect);
            Eigen::Vector3d accTrunkPos = 
                getVect(prefix + "_acc_trunk_pos", vect);
            Eigen::Vector3d posTrunkAxis = 
                getVect(prefix + "_pos_trunk_axis", vect);
            Eigen::Vector3d velTrunkAxis = 
                getVect(prefix + "_vel_trunk_axis", vect);
            Eigen::Vector3d accTrunkAxis = 
                getVect(prefix + "_acc_trunk_axis", vect);
            Eigen::Vector3d posFootPos = 
                getVect(prefix + "_pos_foot_pos", vect);
            Eigen::Vector3d velFootPos = 
                getVect(prefix + "_vel_foot_pos", vect);
            Eigen::Vector3d accFootPos = 
                getVect(prefix + "_acc_foot_pos", vect);
            Eigen::Vector3d posFootAxis = 
                getVect(prefix + "_pos_foot_axis", vect);
            Eigen::Vector3d velFootAxis = 
                getVect(prefix + "_vel_foot_axis", vect);
            Eigen::Vector3d accFootAxis = 
                getVect(prefix + "_acc_foot_axis", vect);
            traj.get("trunk_pos_x").addPoint(
                time, posTrunkPos.x(), velTrunkPos.x(), accTrunkPos.x());
            traj.get("trunk_pos_y").addPoint(
                time, posTrunkPos.y(), velTrunkPos.y(), accTrunkPos.y());
            traj.get("trunk_pos_z").addPoint(
                time, posTrunkPos.z(), velTrunkPos.z(), accTrunkPos.z());
            traj.get("trunk_axis_x").addPoint(
                time, posTrunkAxis.x(), velTrunkAxis.x(), accTrunkAxis.x());
            traj.get("trunk_axis_y").addPoint(
                time, posTrunkAxis.y(), velTrunkAxis.y(), accTrunkAxis.y());
            traj.get("trunk_axis_z").addPoint(
                time, posTrunkAxis.z(), velTrunkAxis.z(), accTrunkAxis.z());
            traj.get("foot_pos_x").addPoint(
                time, posFootPos.x(), velFootPos.x(), accFootPos.x());
            traj.get("foot_pos_y").addPoint(
                time, posFootPos.y(), velFootPos.y(), accFootPos.y());
            traj.get("foot_pos_z").addPoint(
                time, posFootPos.z(), velFootPos.z(), accFootPos.z());
            traj.get("foot_axis_x").addPoint(
                time, posFootAxis.x(), velFootAxis.x(), accFootAxis.x());
            traj.get("foot_axis_y").addPoint(
                time, posFootAxis.y(), velFootAxis.y(), accFootAxis.y());
            traj.get("foot_axis_z").addPoint(
                time, posFootAxis.z(), velFootAxis.z(), accFootAxis.z());
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

        /**
         * Throw exception if given parameter name
         * is not defined
         */
        void checkName(
            const std::string& name) const
        {
            if (!isDefined(name)) {
                throw std::logic_error(
                    "TrajectoryParameters unknown parameter: " 
                    + name);
            }
        }

        /**
         * Recompute all optimized parameter index
         */
        void computeIndex()
        {
            //Count optimized parameters
            //and reset index
            _countOptimized = 0;
            for (auto& it : _container) {
                if (it.second.isOptimized) {
                    it.second.index = _countOptimized;
                    _countOptimized++;
                } else {
                    it.second.index = (size_t)-1;
                }
            }
        }
};

}

#endif

