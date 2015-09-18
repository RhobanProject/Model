#pragma once

#include "Model/Model.hpp"
#include "LegIK/LegIK.hpp"

namespace Leph {

/**
 * HumanoidModelWithToe
 *
 * Inherit Model and implement
 * the feet bounding box
 */
  class HumanoidModelWithToe : public Model
  {
  public:
    HumanoidModelWithToe();

    /**
     * Initialize the model with given
     * Robot type and root updater
     * and enable floating base 6 DOF
     */
    HumanoidModelWithToe(
      const std::string& filename,
      const std::string& frameRoot);

    HumanoidModelWithToe(RBDL::Model & model);
        
    /**
     * Virtual destructor
     */
    virtual ~HumanoidModelWithToe();
        
    /**
     * @Inherit
     * Draw feet bounding box
     */
    void boundingBox(size_t frameIndex, 
                     double& sizeX, double& sizeY, double& sizeZ,
                     Eigen::Vector3d& center) const override;
  };

}
