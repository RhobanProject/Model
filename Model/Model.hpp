#ifndef LEPH_MODEL_HPP
#define LEPH_MODEL_HPP

#include <vector>
#include <map>
#include <string>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Types/VectorLabel.hpp"

namespace Leph {

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

/**
 * Model
 *
 * Simple model wrapper
 * interface for RBDL
 */
class Model
{
    public:

        /**
         * Empty initialization
         */
        Model();

        /**
         * Initialization with URDF file
         */
        Model(const std::string& filename);

        /**
         * Initialize with RBDL model
         */
        Model(const RBDL::Model& model);

        /**
         * Return the number of degrees of freedom
         */
        size_t sizeDOF() const;

        /**
         * Get and set current angular value 
         * of degrees of freedom in radian
         *
         * If setBase is false, degrees of freedom
         * updating base are skipped from assignement
         */
        const VectorLabel& getDOF();
        double getDOF(const std::string& name) const;
        double getDOF(size_t index) const;
        void setDOF(const VectorLabel& vect, bool setBase = true);
        void setDOF(const std::string& name, double value);
        void setDOF(size_t index, double value);

        /**
         * Return a list of the actuated motors of the robot
         * (do not include floating base DOF)
         */
        const std::vector<std::string>& getDOFNames() const;
        const std::vector<std::string>& getActuatedDOFNames() const;

        /**
         * Direct getter and setter to Eigen
         * degree of freedom vector
         */
        const Eigen::VectorXd& getDOFVect() const;
        void setDOFVect(const Eigen::VectorXd& vect);

        /**
         * Import given model DOF into this one
         * (no check on same model are done)
         */
        void importDOF(const Model& model);

        /**
         * Reset all degrees of freedom to zero position
         */
        void setDOFZeros();

        /**
         * Return the degree of freedom name from its
         * given index and inverse mapping
         */
        const std::string& getDOFName(size_t index) const;
        size_t getDOFIndex(const std::string& name) const;
        
        /**
         * Return the number of reference frame
         */
        size_t sizeFrame() const;

        /**
         * Return the name given reference 
         * frame index (from 0 to sizeFrame())
         */
        const std::string& getFrameName(size_t index) const;

        /**
         * Return the frame index form given name
         */
        size_t getFrameIndex(const std::string& name) const;

        /**
         * Compute the given 3d point position expresssed
         * in srcFrameIndex and return the result
         * with respect to dstFrameIndex
         * Default point is 0
         * Current degrees of freedom angular values are used
         */
        Eigen::Vector3d position(
            size_t srcFrameIndex, 
            size_t dstFrameIndex,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
        Eigen::Vector3d position(
            const std::string& srcFrame, 
            const std::string& dstFrame,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());

        /**
         * Compute the rotation matrix
         * The tranformation matrix is expressing vector from
         * destination frame within source frame.
         * Columns vectors are destination unit vectors in
         * sources frame.
         * m(0:3,0) is ux_dst in source frame.
         * Current degrees of freedom angular values are used
         */
        Eigen::Matrix3d orientation(
            size_t srcFrameIndex, 
            size_t dstFrameIndex);
        Eigen::Matrix3d orientation(
            const std::string& srcFrame, 
            const std::string& dstFrame);

        /**
         * Compute from rotation matrix
         * the euler yaw angle associated
         * with rotation of srcFrame arround
         * the Z axis of dstFrame.
         */
        double orientationYaw(
            size_t srcFrameIndex,
            size_t dstFrameIndex);
        double orientationYaw(
            const std::string& srcFrame,
            const std::string& dstFrame);

        /**
         * Return the position of center of mass with
         * respect to given frame
         * Current degrees of freedom angular values are used
         */
        Eigen::Vector3d centerOfMass(size_t frameIndex);
        Eigen::Vector3d centerOfMass(const std::string& frame);

        /**
         * Return the total mass of the Model
         */
        double sumMass();

        /**
         * Compute classical Inverse Dynamics on tree model 
         * and return computed torques for 
         * each degrees of freedom using current position. 
         * Optional current velocity and acceleration can be given. 
         * Default is zeros.
         */
        Eigen::VectorXd inverseDynamics(
            const Eigen::VectorXd& velocity = Eigen::VectorXd(),
            const Eigen::VectorXd& acceleration = Eigen::VectorXd());
        VectorLabel inverseDynamics(
            const VectorLabel& velocity,
            const VectorLabel& acceleration);
        
        /**
         * Compute Inverse Dynamics on a modified closed loop
         * model where given frame index is considered fixed
         * in base coordinates. Computed torques are returned
         * but floating base degrees of freedom are set to zero.
         * Current position is used.
         * If useInfinityNorm is true, infinity norm is minimized to
         * solve the torques underdetermined system with high performance cost.
         * Optional current velocity and acceleration can be given. 
         * Default is zeros.
         */
        Eigen::VectorXd inverseDynamicsClosedLoop(
            size_t fixedFrameIndex,
            bool useInfinityNorm = false,
            const Eigen::VectorXd& velocity = Eigen::VectorXd(),
            const Eigen::VectorXd& acceleration = Eigen::VectorXd());
        Eigen::VectorXd inverseDynamicsClosedLoop(
            const std::string& fixedFrameName,
            bool useInfinityNorm = false,
            const Eigen::VectorXd& velocity = Eigen::VectorXd(),
            const Eigen::VectorXd& acceleration = Eigen::VectorXd());

        /**
         * Return optionaly non zero aligned axis bounding box
         * with respect to given frame base and its half size
         */
        virtual void boundingBox(size_t frameIndex, 
            double& sizeX, double& sizeY, double& sizeZ,
            Eigen::Vector3d& center) const;

        /**
         * Direct access to RBDL model
         */
        const RBDL::Model& getRBDLModel() const;

        /**
         * Convert RBDL body id to frame index and
         * frame index to RBDL body id
         */
        size_t bodyIdToFrameIndex(size_t index) const;
        size_t frameIndexToBodyId(size_t index) const;

        std::vector<std::string> getFrames() const;

    protected:
        
        /**
         * Parse and initilialize RBDL model
         */
        void initializeModel(RBDL::Model& model);

    private:
    
        /**
         * RBDL model instance
         */
        RBDL::Model _model;

        /**
         * Joint index to name 
         * and name to index mapping
         */
        std::vector<std::string> _dofIndexToName;
        std::map<std::string, size_t> _dofNameToIndex;

        /**
         * Current DOF angle values
         * in radian in RBDL Eigen format
         */
        RBDLMath::VectorNd _dofs;

        /**
         * VectorLabel DOF 
         * not sync (used for labels)
         */
        VectorLabel _vectorDOF;

        /**
         * Frame index to name 
         * and name to index mapping
         * And frame index to RBDL id
         */
        std::map<size_t, std::string> _frameIndexToName;
        std::map<std::string, size_t> _frameNameToIndex;
        std::map<size_t, size_t> _frameIndexToId;

        /**
         * A list of all the degrees of freedom of the robot
         * (base excepted)
         */
        std::vector<std::string> _actuatedDOFNames;

        /**
         * Filter body name to joint and frame name
         */
        std::string filterJointName(const std::string& name) const;
        std::string filterFrameName(const std::string& name) const;

        /**
         * Return the real name of given body index
         * and the vitual body hierarchy depth
         * (Handle virtual body for multi DOF joint)
         * Inspired by RBDL/src/rbdl_utils.cc
         */
        std::string getRBDLBodyName(size_t bodyId, 
            unsigned int& virtualDepth) const;

        /**
         * Add a degree of freedom with given name
         */
        void addDOF(const std::string& name);

        /**
         * Update values from RBDL Eigen DOF vector
         * to VectorLabel
         */
        void loadEigenToLabel();

        /**
         * Update given Eigen vector with set values
         * in VectorLabel
         * Base DOF are skipped if setBase is true
         */
        void loadLabelToEigen(const VectorLabel& vect, 
            Eigen::VectorXd& dst, bool setBase);

        /**
         * Direct access for InverseKinematics class
         */
        friend class InverseKinematics;
};

}

#endif

