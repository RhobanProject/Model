#ifndef LEPH_EULER_H
#define LEPH_EULER_H

#include <Eigen/Dense>

namespace Leph {

/**
 * All combinations
 * of Euler angles types
 * in same order as rotation application
 *
 * EulerYawPitchRoll is built as
 * Roll * Pitch * Yaw.
 */
enum EulerType {
    EulerYawPitchRoll,
    EulerYawRollPitch,
    EulerRollPitchYaw,
    EulerRollYawPitch,
    EulerPitchRollYaw,
    EulerPitchYawRoll,
};

/**
 * Convert given Euler angles of given
 * convention type to rotation matrix
 */
inline Eigen::Matrix3d EulerToMatrix(
    const Eigen::Vector3d angles, EulerType eulerType)
{
    Eigen::Quaternion<double> quat;
    switch (eulerType) {
        case EulerYawPitchRoll: {
            Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
            quat = rollRot * pitchRot * yawRot;
        }
        break;
        case EulerYawRollPitch: {
            Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
            quat = pitchRot * rollRot * yawRot;
        }
        break;
        case EulerRollPitchYaw: {
            Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
            quat = yawRot * pitchRot * rollRot;
        }
        break;
        case EulerRollYawPitch: {
            Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
            quat = pitchRot * yawRot * rollRot;
        }
        break;
        case EulerPitchRollYaw: {
            Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
            quat = yawRot * rollRot * pitchRot;
        }
        break;
        case EulerPitchYawRoll: {
            Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
            quat = rollRot * yawRot * pitchRot;
        }
        break;
    }
    return quat.matrix();
}

}

#endif

