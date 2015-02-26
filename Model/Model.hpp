#ifndef LEPH_MODEL_HPP
#define LEPH_MODEL_HPP

#include <vector>
#include <map>
#include <string>
#include <rbdl/rbdl.h>
#include <urdfreader/rbdl_urdfreader.h>
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
         * Initialization with URDF file
         */
        Model(const std::string& filename);

        /**
         * Return the number of degrees of freedom
         */
        size_t sizeDOF() const;

        /**
         * Get and set current angular value 
         * of degrees of freedom in degrees
         */
        VectorLabel getDOF() const;
        void setDOF(const VectorLabel& vect);
        void setDOF(const std::string& name, double value);
        
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
         * Index to RDBL convertion are skipped if frameConvertion
         * is false
         */
        Eigen::Vector3d position(
            size_t srcFrameIndex, size_t dstFrameIndex,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
        Eigen::Vector3d position(
            const std::string& srcFrame, const std::string& dstFrame,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());

        /**
         * Compute the rotation matrix
         * used to expressed the srcFrameIndex unit 
         * coordinates to dstFrameIndex coordinates
         * Current degrees of freedom angular values are used
         * Index to RDBL convertion are skipped if frameConvertion
         * is false
         */
        Eigen::Matrix3d orientation(
            size_t srcFrameIndex, size_t dstFrameIndex);
        Eigen::Matrix3d orientation(
            const std::string& srcFrame, const std::string& dstFrame);

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
         * Direct access to RBDL model
         */
        const RBDL::Model& getRBDLModel() const;

        /**
         * Convert RBDL body id to frame index
         */
        size_t bodyIdToFrameIndex(size_t index) const;

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
         * in degrees in VectorLabel format and Eigen format
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
         * Build Eigen degree of freedom 
         * vector from VectorLabel
         */
        RBDLMath::VectorNd buildDOFVector() const;
};

}

#endif

