#ifndef LEPH_ODOMETRYMODEL_HPP
#define LEPH_ODOMETRYMODEL_HPP

#include <Eigen/Dense>
#include "Model/HumanoidFixedModel.hpp"

namespace Leph {

/**
 * OdometryModel
 *
 * Compute and correct relative odometry 
 * displacement with simple linear model.
 */
class OdometryModel
{
    public:

        /**
         * OdometryModel regression type
         */
        enum OdometryModelType {
            CorrectionLinear,
            CorrectionLinearWithAzimuth,
            CorrectionCubic,
        };

        /**
         * Initialization
         */
        OdometryModel(OdometryModelType type);

        /**
         * Return instance Correctin type
         */
        OdometryModelType getType() const;

        /**
         * Reset to zero integrated state
         * and mark internal data to be re initialized
         */
        void reset();

        /**
         * Update with given input pose state or input Model
         * the internal data and compute corrected odometry.
         */
        void update(
            const Eigen::Vector3d& pose, 
            Leph::HumanoidFixedModel::SupportFoot supportFoot);
        void update(
            HumanoidFixedModel& model);

        /**
         * Read/Write access to 
         * odometry parameters
         */
        const Eigen::VectorXd& parameters() const;
        Eigen::VectorXd& parameters();

        /**
         * Return current corrected odometry state
         * [x,y,theta]
         */
        const Eigen::Vector3d& state() const;
        
        /**
         * Compute odometry displacement
         * vector from state1 to state2
         */
        Eigen::Vector3d odometryDiff(
            const Eigen::Vector3d& state1, 
            const Eigen::Vector3d& state2) const;

        /**
         * Integrate given odometry diff vector
         * to given state and update it
         */
        void odometryInt(
            const Eigen::Vector3d& diff,
            Eigen::Vector3d& state) const;

    private:

        /**
         * Seleted correction type
         */
        const OdometryModelType _type;

        /**
         * If false, the next update() will
         * initialize internal data to match
         * current input Model state
         */
        bool _isInitialized;

        /**
         * Odometry parameters
         * 0: dX = offset
         * 1: dX = dx
         * 2: dX = dx^2
         * 3: dX = dx^3
         * 4: dX = dy
         * 5: dX = dy^2
         * 6: dX = dy^3
         * 7: dY = offset
         * 8: dY = dx
         * 9: dY = dx^2
         *10: dY = dx^3
         *11: dY = dy
         *12: dY = dy^2
         *13: dY = dy^3
         */
        Eigen::VectorXd _odometryParameters;
        
        /**
         * Last seen support foot 
         * of input Model.
         * Use to detected support swap.
         */
        Leph::HumanoidFixedModel::SupportFoot _support;

        /**
         * Input Model robot pose (self in 
         * origin) in world frame at last 
         * right to left support foot swap
         */
        Eigen::Vector3d _last;

        /**
         * Output corrected robot pose in
         * world frame at last right to left
         * support foot swap.
         */
        Eigen::Vector3d _state;

        /**
         * Output corrected robot pose 
         * in world frame integrated at each
         * update
         */
        Eigen::Vector3d _corrected;
};

}

#endif

