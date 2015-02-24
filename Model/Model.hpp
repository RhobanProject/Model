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
            size_t srcFrameIndex, size_t dstFrameIndex,
            const Eigen::Vector3d& point = Eigen::Vector3d::Zero());

        /**
         * Direct access to RBDL model
         */
        const RBDL::Model& getRBDLModel() const;

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
};

}

#endif

