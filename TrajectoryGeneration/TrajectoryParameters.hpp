#ifndef LEPH_TRAJECTORYPARAMETERS_HPP
#define LEPH_TRAJECTORYPARAMETERS_HPP

#include <string>
#include <map>
#include <Eigen/Dense>

namespace Leph {

/**
 * TrajectoryParameters
 */
class TrajectoryParameters
{
    public:

        /**
         *
         */
        void add(bool isOptimized, const std::string& name, double value);
        void add(bool isOptimized, const std::string& name, const Eigen::Vector3d& vect);

        /**
         *
         */
        double get(const std::string& name);
        Eigen::Vector3d getVect(const std::string& name);

        /**
         *
         */
        const Eigen::VectorXd& getParameters();
        Eigen::VectorXd& getParameters();

        /**
         *
         */
        void write(const std::string& name);

        /**
         *
         */
        void read(const std::string& name);

    private:

        /**
         *
         */
        std::map<std::string, size_t> _notOptimized;
        std::map<std::string, size_t> _isOptimized;
};

}

#endif

