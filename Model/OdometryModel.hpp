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
            CorrectionIdentity,
            CorrectionScalarX,
            CorrectionScalarXY,
            CorrectionScalarXYZ,
            CorrectionProportionalXY,
            CorrectionProportionalXYZ,
            CorrectionLinearSimpleXY,
            CorrectionLinearSimpleXYZ,
            CorrectionLinearFullXY,
            CorrectionLinearFullXYZ,
        };

        /**
         * Initialization
         */
        OdometryModel(OdometryModelType type);

        /**
         * Return instance Correction type
         */
        OdometryModelType getType() const;

        /**
         * Reset to zero or given pose integrated state
         * and mark internal data to be re initialized
         */
        void reset();
        void reset(const Eigen::Vector3d& pose);

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
         * Update pose state by integrating given
         * relative displacement between two right to left
         * support foot transition (dX, dY, dTheta) 
         * (meter, radian).
         */
        void updateFullStep(
            const Eigen::Vector3d& deltaPose);

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

        /**
         * Correct and return given relative displacement 
         * [dX,dY,dTheta] using current model parameters.
         */
        Eigen::Vector3d correctiveModel(
            const Eigen::Vector3d& diff) const;

        /**
         * Return minimum and maximum (indicative)
         * constrains for current Odometry model
         * parameters
         */
        const Eigen::VectorXd& parameterLowerBounds() const;
        const Eigen::VectorXd& parameterUpperBounds() const;

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
         * Odometry parameters depending
         * on OdometryModelType.
         * Lower and upper parameter bounds.
         */
        Eigen::VectorXd _odometryParameters;
        Eigen::VectorXd _odometryLowerBounds;
        Eigen::VectorXd _odometryUpperBounds;
        
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

