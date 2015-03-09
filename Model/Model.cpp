#include <stdexcept>
#include "Model/Model.hpp"

namespace Leph {
        
Model::Model(const std::string& filename) :
    _model(),
    _dofIndexToName(),
    _dofNameToIndex(),
    _dofs(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId()
{
    //URDF loading
    if (!RBDL::Addons::URDFReadFromFile(filename.c_str(), &_model, false)) {
        std::runtime_error("Model unable to load URDF file: " + filename);
    }

    //Build name-index joint mapping 
    //and VectorLabel structure
    for (size_t i=1;i<_model.mBodies.size();i++) {
        unsigned int virtualDepth = 0;
        std::string filteredName = filterJointName(
            getRBDLBodyName(i, virtualDepth));
        //Handle special case of 6 virtual bodies added by
        //the floating joint
        if (virtualDepth == 5) {
            addDOF(filteredName + " Tx");
            addDOF(filteredName + " Ty");
            addDOF(filteredName + " Tz");
            addDOF(filteredName + " roll");
            addDOF(filteredName + " pitch");
            addDOF(filteredName + " yaw");
            i += 5;
            continue;
        } else if (virtualDepth > 0) {
            std::cout << "*** " << i << " :: " << virtualDepth << std::endl;
            throw std::logic_error(
                "Model virtual body name not implemented");
        }
        addDOF(filteredName);
    }

    //Build name-index frame mapping
    size_t index = 0;
    for (const auto& name : _model.mBodyNameMap) {
        if (name.second == 0) continue;
        std::string filteredName = filterFrameName(name.first);
        _frameIndexToName[index] = filteredName;
        _frameNameToIndex[filteredName] = index;
        _frameIndexToId[index] = name.second;
        index++;
    }
    _frameIndexToName[index] = "origin";
    _frameNameToIndex["origin"] = index;
    _frameIndexToId[index] = 0;
}
        
size_t Model::sizeDOF() const
{
    return _model.dof_count;
}
        
const VectorLabel& Model::getDOF()
{
    loadEigenToLabel();
    return _vectorDOF;
}
double Model::getDOF(const std::string& name) const
{
    return _dofs(_dofNameToIndex.at(name));
}
void Model::setDOF(const VectorLabel& vect)
{
    _vectorDOF.mergeInter(vect);
    loadLabelToEigen();
}
void Model::setDOF(const std::string& name, double value)
{
    _dofs(_dofNameToIndex.at(name)) = value;
}

size_t Model::sizeFrame() const
{
    return _frameIndexToName.size();
}

const std::string& Model::getFrameName(size_t index) const
{
    return _frameIndexToName.at(index);
}
        
size_t Model::getFrameIndex(const std::string& name) const
{
    return _frameNameToIndex.at(name);
}
        
Eigen::Vector3d Model::position(
    size_t srcFrameIndex, size_t dstFrameIndex,
    const Eigen::Vector3d& point)
{
    //Convert to body id
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    dstFrameIndex = _frameIndexToId.at(dstFrameIndex);

    //Compute transformation from body1 to base and base to body2
    RBDLMath::Vector3d ptBase = RBDL::CalcBodyToBaseCoordinates(
        _model, _dofs, srcFrameIndex, point);
    RBDLMath::Vector3d ptBody = RBDL::CalcBaseToBodyCoordinates(
        _model, _dofs, dstFrameIndex, ptBase);

    return ptBody;
}
Eigen::Vector3d Model::position(
    const std::string& srcFrame, const std::string& dstFrame,
    const Eigen::Vector3d& point)
{
    return position(
        getFrameIndex(srcFrame), 
        getFrameIndex(dstFrame),
        point);
}
        
Eigen::Matrix3d Model::orientation(
    size_t srcFrameIndex, size_t dstFrameIndex)
{
    //Convert to body id
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    dstFrameIndex = _frameIndexToId.at(dstFrameIndex);
    
    RBDLMath::Matrix3d transform1 = CalcBodyWorldOrientation(
        _model, _dofs, srcFrameIndex);
    RBDLMath::Matrix3d transform2 = CalcBodyWorldOrientation(
        _model, _dofs, dstFrameIndex);

    return transform1*transform2.transpose();
}
Eigen::Matrix3d Model::orientation(
    const std::string& srcFrame, const std::string& dstFrame)
{
    return orientation(
        getFrameIndex(srcFrame), 
        getFrameIndex(dstFrame));
}
        
Eigen::Vector3d Model::centerOfMass(size_t frameIndex)
{
    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(_model, _dofs, _dofs, mass, com);

    return position(0, frameIndex, com);
}
Eigen::Vector3d Model::centerOfMass(const std::string& frame)
{
    return centerOfMass(getFrameIndex(frame));
}
double Model::sumMass()
{
    RBDLMath::VectorNd Q(sizeDOF());
    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(_model, Q, Q, mass, com);

    return mass;
}

void Model::boundingBox(size_t frameIndex, 
    double& sizeX, double& sizeY, double& sizeZ,
    Eigen::Vector3d& center) const
{
    //Return null box for default behaviour
    (void)frameIndex;
    sizeX = 0.0;
    sizeY = 0.0;
    sizeZ = 0.0;
    center = Eigen::Vector3d::Zero();
}

const RBDL::Model& Model::getRBDLModel() const
{
    return _model;
}

size_t Model::bodyIdToFrameIndex(size_t index) const
{
    for (const auto& mapping : _frameIndexToId) {
        if (mapping.second == index) {
            return mapping.first;
        }
    }
    throw std::logic_error("Model invalid RBDL id");
}
size_t Model::frameIndexToBodyId(size_t index) const
{
    return _frameIndexToId.at(index);
}

std::string Model::filterJointName(const std::string& name) const
{
    std::string filtered = name;
    size_t pos = filtered.rfind("_link");
    if (pos != std::string::npos) {
        filtered = filtered.substr(0, pos);
    } 

    for (size_t i=0;i<filtered.length();i++) {
        if (filtered[i] == '_') {
            filtered[i] = ' ';
        }
    }

    return filtered;
}
        
std::string Model::filterFrameName(const std::string& name) const
{
    std::string filtered = name;
    size_t pos = filtered.rfind("_link");
    if (pos != std::string::npos) {
        filtered = filtered.substr(0, pos);
    } 
    
    for (size_t i=0;i<filtered.length();i++) {
        if (filtered[i] == '_') {
            filtered[i] = ' ';
        }
    }
    
    return filtered;
}
        
std::string Model::getRBDLBodyName(size_t bodyId, 
    unsigned int& virtualDepth) const
{
    //If this is a virtual body that was added by a multi dof joint
    if (_model.mBodies[bodyId].mMass < 0.001) {
        //If there is not a unique child we do not know what to do
        if (_model.mu[bodyId].size() != 1) {
            return "";
        }
        virtualDepth++;
        return getRBDLBodyName(_model.mu[bodyId][0], virtualDepth);
    }

    return _model.GetBodyName(bodyId);
}
        
void Model::addDOF(const std::string& name)
{
    _vectorDOF.append(name, 0.0);
    _dofNameToIndex[name] = _dofIndexToName.size();
    _dofIndexToName.push_back(name);
    _dofs = RBDLMath::VectorNd::Zero(_vectorDOF.size());
}
        
void Model::loadEigenToLabel()
{
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        const std::string& name = _dofIndexToName.at(i);
        _vectorDOF(_dofIndexToName.at(i)) = _dofs(i);
    }
}
void Model::loadLabelToEigen()
{
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        const std::string& name = _dofIndexToName.at(i);
        _dofs(i) = _vectorDOF(_dofIndexToName.at(i));
    }
}
 
}

