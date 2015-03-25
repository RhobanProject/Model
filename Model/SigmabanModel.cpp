#include "Model/RBDLRootUpdate.h"
#include "Model/SigmabanModel.hpp"

namespace Leph {

SigmabanModel::SigmabanModel(const std::string& frameRoot) :
    Model()
{
    //Load model from URDF file
    RBDL::Model modelOld;
    if (!RBDL::Addons::URDFReadFromFile(
        "sigmaban.urdf", &modelOld, false)
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
    Model::initilializeModel(modelNew);
}
        
SigmabanModel::~SigmabanModel()
{
}
        
void SigmabanModel::boundingBox(size_t frameIndex, 
    double& sizeX, double& sizeY, double& sizeZ,
    Eigen::Vector3d& center) const
{
    if (Model::getFrameName(frameIndex) == "left foot tip") {
        sizeX = 0.062495;
        sizeY = 0.039995;
        sizeZ = 0.01;
        center = Eigen::Vector3d(0.001845, 0.002495, 0.01);
    } else if (Model::getFrameName(frameIndex) == "right foot tip") {
        sizeX = 0.062495;
        sizeY = 0.039995;
        sizeZ = 0.01;
        center = Eigen::Vector3d(0.001845, -0.002495, 0.01);
    } else {
        Model::boundingBox(frameIndex, 
            sizeX, sizeY, sizeZ, center);
    }
}

}

