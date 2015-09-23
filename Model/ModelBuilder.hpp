#pragma once

#include "Model/Model.hpp"

namespace Leph {

  Leph::Model generateGrobanWithToe(bool floatingBase);

  /**
   * Adding virtual Degrees of freedom, those degrees have a body with
   * 'almost no weight' (rbdl does not allow weightless bodies) and are
   * used only to describe the structure.
   */
  //No rotation
  void addVirtualDOF(RBDL::Model & model,
                     const std::string & srcFrame,
                     const RBDL::Joint & joint,
                     const Eigen::Vector3d & toChild,
                     const std::string & childFrame);

  void addVirtualDOF(RBDL::Model & model,
                     const std::string & srcFrame,
                     const RBDL::Joint & joint,
                     RBDLMath::SpatialTransform st,
                     const std::string & childFrame);

  /**
   * Add a fixed body to the body with the given name using a fixed link
   */

  // No spatial transformation
  void addFixedBody(RBDL::Model & model,
                    const std::string & srcFrame,
                    const std::string & bodyName,
                    const std::string & childFrame);


  // No rotation
  void addFixedBody(RBDL::Model & model,
                    const std::string & srcFrame,
                    const std::string & bodyName,
                    const Eigen::Vector3d & offset,
                    const std::string & childFrame);
  
  void addFixedBody(RBDL::Model & model,
                    const std::string & srcFrame,
                    const std::string & bodyName,
                    RBDLMath::SpatialTransform st,
                    const std::string & childFrame);
}
