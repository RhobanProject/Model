#ifndef LEPH_MOTORMODEL_HPP
#define LEPH_MOTORMODEL_HPP

#include <Eigen/Dense>

namespace Leph {

/**
 * MotorModel
 *
 * Simple static class for modelling the
 * Dynamixel MX-64 servo motors and its
 * operation bounds
 */
class MotorModel
{
    public:

        /**
         * Return the maximum control
         * voltage provided by the motor
         */
        static double maxVoltage();

        /**
         * Return the maximum stall torque
         * and velocity
         */
        static double maxTorque();
        static double maxVelocity();

        /**
         * Return min and max torque bounds
         * given current motor velocity
         */
        static double maxTorque(double velocity);
        static double minTorque(double velocity);

        /**
         * Compute the control voltage provided
         * to follow given velocity and torque
         */
        static double voltage(double velocity, double torque);
        static Eigen::VectorXd voltage(
            const Eigen::VectorXd& velocity, const Eigen::VectorXd& torque);

    private:

        /**
         * Motor model parameters
         */
        //(V)
        static constexpr double _uMax = 12.0;
        //(V.s/rad)
        static constexpr double _ke = 1.399;
        //(N.m/A)
        static constexpr double _kt = 1.399;
        //(ohm)
        static constexpr double _r = 4.07;
};

}

#endif

