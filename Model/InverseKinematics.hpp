#ifndef LEPH_INVERSEKINEMATICS_HPP
#define LEPH_INVERSEKINEMATICS_HPP

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <Eigen/Dense>
#include "EigenLevenbergMarquardt/LevenbergMarquardt"
#include "Model/Model.hpp"

namespace Leph {

/**
 * InverseKinematics
 *
 * Implement inverse kinematrics for Leph::Model
 * using Eigen implementation of Levenberg-Marquardt
 * algorithm and bounds (box) constraints
 * Hold RBDL model computation, degree of freedom subset,
 * joint limit and geometric target sets
 */
class InverseKinematics : public Eigen::DenseFunctor<double>
{
    public:

        /**
         * Enum for scalar target axis
         */
        enum TargetAxis {
            AxisX,
            AxisY,
            AxisZ
        };

        /**
         * Initialization with robot Model instance
         */
        InverseKinematics(Model& model);
        
        /**
         * Add given degree of freedom name
         * for beeing optimized
         */
        void addDOF(const std::string& name);


        /**
         * Return a VectorLabel associating the name of the DOF with their values
         */
        VectorLabel getNamedDOFSubset();

        /**
         * Set lower and upper bound limit value
         * or disable constraint for given degree of freedom
         */
        /**
         * TODO
         * Box constraints implementation is lacking of active constraint
         * management in order to update the jacobian with respect to
         * active contraints
         * For now, only gradient "cut" is implemented to force algorithm
         * step to remain inside allowed DOF range
         * When a mimit range is reached, the constraint become active
         * and associated line in jacobian need to be trim
         * TODO
         */
        void setLowerBound(const std::string& name, double value);
        void setUpperBound(const std::string& name, double value);
        void clearLowerBound(const std::string& name);
        void clearUpperBound(const std::string& name);

        /**
         * Add a target vector position with given targetName.
         * The given point in srcFrame is constrained
         * to equals the target vector position 
         * in world frame
         * Target initial position is set to 
         * current model state
         */
        void addTargetPosition(
            const std::string& targetName,
            const std::string& srcFrame,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());

        /**
         * Add a target orientation with given targetName.
         * The given srcFrame is constrained
         * to equals the target rotation matrix
         * in world frame
         * Target initial position is set to 
         * current model state
         */
        void addTargetOrientation(
            const std::string& targetName,
            const std::string& srcFrame);

        /**
         * Add a target vector position with given targetName.
         * The given point in srcFrame is constrained
         * to equals the target vector position only along
         * given axis in world frame
         * Target initial position is set to 
         * current model state
         */
        void addTargetScalar(
            const std::string& targetName,
            const std::string& srcFrame,
            TargetAxis axis,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());

        /**
          * Add a target value for a given DOF
          */
        void addTargetDOF(const std::string& targetName,
                          const std::string& dofName);

        /**
         * Add a target on model center of mass
         */
        void addTargetCOM();

        /**
         * Access and update target position vector, 
         * orientation matrix, scalar axis value and dof
         * of given targetName
         * Access to center of mass target
         */
        Eigen::Vector3d& targetPosition(
            const std::string& targetName);
        Eigen::Matrix3d& targetOrientation(
            const std::string& targetName);
        double& targetScalar(
            const std::string& targetName);
        double& targetDOF(const std::string& targetName);
        Eigen::Vector3d& targetCOM();

        /**
         * Access and update weight of the different targets
         */
        Eigen::Vector3d& weightPosition(const std::string& targetName);
        double& weightOrientation(const std::string& targetName);
        double& weightScalar(const std::string& targetName);
        double& weightDOF(const std::string& targetName);
        Eigen::Vector3d& weightCOM();

        /**
         * Return last computed target error
         * for position vector, orientation,
         * scalar and center of mass
         */
        double errorPosition(
            const std::string& targetName) const;
        double errorOrientation(
            const std::string& targetName) const;
        double errorScalar(
            const std::string& targetName) const;
        double errorDOF(const std::string& targetName) const;
        double errorCOM() const;

        /**
         * Return the sum of all targets errors
         */
        double errorSum() const;

        /**
         * Apply small random perturbation of given
         * amplitude to current degree of freedom subset
         * to help convergence in singular configuration
         */
        void randomDOFNoise(double ampl = 0.01);

        /**
         * Run the inverse kinematics optimization
         * using Eigen Levenberg Marquardt implementation
         * and update model degree of freedom
         *
         * Parameters are convergence criterion (only one has
         * to be meet for stopping)
         * tolerance is the requested precision. Increase it
         * as long as error is in acceptable range
         * maxEvaluation is the maximum number of model evaluation
         */
        void run(double tolerance, unsigned int maxEvaluation);

        /**
         * Return the number of registered
         * degree of freedom and target function
         * to minimize
         */
        size_t sizeDOF() const;
        size_t sizeTarget() const;

        /**
         * Implement Eigen LevenbergMarquardt 
         * functor interface.
         * Return the number of degree of freedom
         * and min of target function to minimize
         * and inputs
         */
        size_t inputs() const;
        size_t values() const;

        /**
         * Implement Eigen LevenbergMarquardt 
         * functor interface.
         * Compute the function vector fvec to minimize and
         * its jacobian fjac at given dofs state
         * Also compute projected gradient onto box
         * constraint
         */
        int operator()(const Eigen::VectorXd& dofs, 
            Eigen::VectorXd& fvec);
        int df(const Eigen::VectorXd& dofs, 
            Eigen::MatrixXd& fjac);
        void gradientProjection(const Eigen::VectorXd& state,
            Eigen::VectorXd& gradient);

        /**
         * Return current used degree 
         * of freedrom vector subset
         */
        const Eigen::VectorXd& getDOFSubset();

        /**
         * Export to internal model
         * given defree of freedom subset state
         */
        void setDOFSubset(const Eigen::VectorXd& dofs);

        /**
         * Direct access to registered degrees of freedom
         * lower and upper bounds if defined
         */
        const std::vector<double>& getLowerBounds() const;
        const std::vector<double>& getUpperBounds() const;
        const std::vector<bool>& getIsLowerBounds() const;
        const std::vector<bool>& getIsUpperBounds() const;

    private:

        /**
         * Struct for position, orientation, scalar and DOF target
         */
        struct TargetPosition {
            std::string name;
            size_t bodyId;
            Eigen::Vector3d point;
            Eigen::Vector3d target;
            double error;
            Eigen::Vector3d weight;
        };
        struct TargetOrientation {
            std::string name;
            size_t bodyId;
            Eigen::Matrix3d target;
            bool isPosTarget;
            Eigen::Vector3d posTarget;
            double error;
            double weight;
        };
        struct TargetScalar {
            std::string name;
            size_t bodyId;
            Eigen::Vector3d point;
            TargetAxis axis;
            double target;
            double error;
            double weight;
        };
        struct TargetDOF {
            std::string name;
            size_t subsetIndex;
            double target;
            double error;
            double weight;
        };

        /**
         * Return a reference to the target structure from its name.
         * Throw a std::out_of_range exception with an explicit message
         * if targetName is unknown
         */
        const struct TargetPosition&    targetPositionRef   (const std::string & targetName) const;
        const struct TargetOrientation& targetOrientationRef(const std::string & targetName) const;
        const struct TargetScalar&      targetScalarRef     (const std::string & targetName) const;
        const struct TargetDOF&         targetDOFRef        (const std::string & targetName) const;
        struct TargetPosition&          targetPositionRef   (const std::string & targetName);
        struct TargetOrientation&       targetOrientationRef(const std::string & targetName);
        struct TargetScalar&            targetScalarRef     (const std::string & targetName);
        struct TargetDOF&               targetDOFRef        (const std::string & targetName);

        /**
         * Leph::Model interface to RBDL
         */
        Model* _model;

        /**
         * Mapping between index of internal
         * subset of DOF and global model DOF index
         * and inverse
         */
        std::vector<size_t> _subsetIndexToGlobal;
        std::map<size_t, size_t> _globalIndexToSubset;

        /**
         * Current subset DOF values
         */
        Eigen::VectorXd _dofs;

        /**
         * Complete DOF values for RBDL calls
         */
        Eigen::VectorXd _allDofs;

        /**
         * Lower and upper box bounds values
         * and flag indicating if bound is enabled
         */
        std::vector<double> _lowerBounds;
        std::vector<double> _upperBounds;
        std::vector<bool> _isLowerBounds;
        std::vector<bool> _isUpperBounds;

        /**
         * Target position, orientation, scalar and DOF
         * container indexed by their name
         */
        std::map<std::string, TargetPosition>    _targetPositions;
        std::map<std::string, TargetOrientation> _targetOrientations;
        std::map<std::string, TargetScalar>      _targetScalars;
        std::map<std::string, TargetDOF>         _targetDOFs;

        /**
         * Special target case of center of mass
         */
        bool _isTargetCOM;
        double _errorCOM;
        Eigen::Vector3d _targetCOM;
        Eigen::Vector3d _weightCOM;

        /**
         * Last error sum
         */
        double _errorSum;

        /**
         * Update complete _allDofs vector with given 
         * DOF subset
         */
        void updateAllDOF(const Eigen::VectorXd& dofs);

        /**
         * Compute center of mass jacobian using
         * current DOF
         */
        void comJacobian(RBDLMath::MatrixNd& fjac, size_t index,
                         const Eigen::Vector3d & weight);

        /**
         * Import and export from and to RBDL model
         * all and subset of DOF
         */
        void importDOF();
        void exportDOF();
};

}

#endif

