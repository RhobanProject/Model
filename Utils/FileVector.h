#ifndef LEPH_FILEVECTOR_HPP
#define LEPH_FILEVECTOR_HPP

#include <Eigen/Dense>

namespace Leph {

/**
 * Write the given vector into
 * given filename
 */
void WriteVector(
    const std::string& filename, 
    const Eigen::VectorXd& vect);

/**
 * Read from given filename a Vector
 * and return it
 */
Eigen::VectorXd ReadVector(
    const std::string& filename);

}

#endif

