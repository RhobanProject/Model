#include "Model/RBDLRootUpdate.h"
#include "Model/HumanoidModelWithToe.hpp"

namespace Leph {

  HumanoidModelWithToe::HumanoidModelWithToe(): Model()
  {
  }

  HumanoidModelWithToe::HumanoidModelWithToe(RBDL::Model & model)
  {
    Model::initializeModel(model);
  }

  HumanoidModelWithToe::HumanoidModelWithToe(
    const std::string& filename,
    const std::string& frameRoot) :
    Model()
  {
    //Load model from URDF file
    RBDL::Model modelOld;
    if (!RBDL::Addons::URDFReadFromFile(
          filename.c_str(), &modelOld, false)
      ) {
      std::runtime_error("Model unable to load URDF file");
    }

    //Select new RBDL body id root
    size_t frameRootId;
    if (frameRoot == "ROOT") {
      frameRootId = 0;
    } else {
      Leph::Model wrappedModelNew(modelOld);
      frameRootId = wrappedModelNew.frameIndexToBodyId(
        wrappedModelNew.getFrameIndex(frameRoot));
    }

    //Update old urdf model with new root frame
    RBDL::Model modelNew = 
      Leph::RBDLRootUpdate(modelOld, frameRootId, true);
    //Initialize base model
    Model::initializeModel(modelNew);

  }
        
  HumanoidModelWithToe::~HumanoidModelWithToe()
  {
  }
        
  void HumanoidModelWithToe::boundingBox(size_t frameIndex, 
                                         double& sizeX, double& sizeY, double& sizeZ,
                                         Eigen::Vector3d& center) const
  {
    if (Model::getFrameName(frameIndex).find("arch_tip") != std::string::npos) {
      sizeX = 0.125 / 2.0;
      sizeY = 0.092 / 2.0;
      sizeZ = 0.02  / 2.0;
      center = Eigen::Vector3d(0.0, 0.0, sizeZ / 2);
    } else if (Model::getFrameName(frameIndex).find("toe_tip") != std::string::npos) {
      sizeX = 0.032 / 2.0;
      sizeY = 0.110 / 2.0;
      sizeZ = 0.02  / 2.0;
      center = Eigen::Vector3d(0.0, 0.0, sizeZ / 2);
    } else {
      Model::boundingBox(frameIndex, 
                         sizeX, sizeY, sizeZ, center);
    }
  }

}

