#ifndef LEPH_AXISANGLE_H
#define LEPH_AXISANGLE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

namespace Leph {

/**
 * Axis angle representation. 
 * The direction is the rotation axis and 
 * the norm is the angle in radian.
 */

/**
 * Compute and return the rotation matrix with given
 * axis. Identity is returned if null vector is given.
 * Rotation angle have to be in -M_PI/2.0:M_PI/2.0
 */
inline Eigen::Matrix3d AxisToMatrix(const Eigen::Vector3d& axis)
{
    double theta = axis.norm();
    if (fabs(theta) > M_PI/2.0) {
        throw std::logic_error("AxisAngle unbound angle (in -M_PI/2:M_PI)");
    }
    if (theta <= 0.0) {
        return Eigen::Matrix3d::Identity();
    } else {
        Eigen::Vector3d vect = axis.normalized();
        Eigen::AngleAxisd axisAngle(theta, vect);
        return axisAngle.matrix();
    }
}

/**
 * Convert and return the rotation axis vector from
 * given rotation matrix.
 * Null vector is returned if identity matrix is given.
 * No check is done on input matrix format.
 */
inline Eigen::Vector3d MatrixToAxis(const Eigen::Matrix3d& mat)
{
    //Skew is the anti-symetric matrix
    Eigen::Matrix3d skew = mat -mat.transpose();
    //Rotation axis extraction
    Eigen::Vector3d axis;
    axis(0) = skew(2, 1);
    axis(1) = skew(0, 2);
    axis(2) = skew(1, 0);
    //Compute rotation angle        
    if (axis.norm() > 0.00001) {
        double theta = std::asin(axis.norm()/2.0);
        return theta*axis.normalized();
    } else {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
}

}

#endif

