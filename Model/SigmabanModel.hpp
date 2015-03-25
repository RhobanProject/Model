#ifndef LEPH_SIGMABANMODEL_HPP
#define LEPH_SIGMABANMODEL_HPP

#include "Model/Model.hpp"

namespace Leph {

/**
 * SigmabanModel
 *
 * Inherit Model and implement
 * Sigmaban feet bounding box
 */
class SigmabanModel : public Model
{
    public:

        /**
         * Initialize the model with root updater
         * and enable foating base 6 DOF
         */
        SigmabanModel(const std::string& frameRoot);
        
        /**
         * Destructor
         */
        virtual ~SigmabanModel();
        
        /**
         * @Inherit
         * Draw feet bounding box
         */
        void boundingBox(size_t frameIndex, 
            double& sizeX, double& sizeY, double& sizeZ,
            Eigen::Vector3d& center) const override;
};

}

#endif

