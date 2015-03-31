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
        Model(RBDL::Model& model);

        /**
         * Return the number of degrees of freedom
         */
        size_t sizeDOF() const;

        /**
         * Get and set current angular value 
         * of degrees of freedom in radian
         */
        const VectorLabel& getDOF();
        double getDOF(const std::string& name) const;
        void setDOF(const VectorLabel& vect);
        void setDOF(const std::string& name, double value);

        /**
         * Import given model DOF into this one
         * (no check on same model are done)
         */
        void importDOF(Model& model);

        /**
         * Reset all degrees of freedom to zero position
         */
        void setDOFZeros();
        
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

    protected:
        
        /**
         * Parse and initilialize RBDL model
         */
        void initilializeModel(RBDL::Model& model);

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
         * Direct access for InverseKinematics class
         */
        friend class InverseKinematics;
};

}

#endif

