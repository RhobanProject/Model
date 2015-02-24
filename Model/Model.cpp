#include <stdexcept>
#include "Model/Model.hpp"

namespace Leph {
        
Model::Model(const std::string& filename) :
    _model(),
    _dofIndexToName(),
    _dofNameToIndex(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId()
{
    //URDF loading
    if (!RBDL::Addons::read_urdf_model(filename.c_str(), &_model, false)) {
        std::runtime_error("Model unable to load URDF file: " + filename);
    }

    //Build name-index joint mapping 
    //and VectorLabel structure
    for (size_t i=1;i<_model.mBodies.size();i++) {
        std::string filteredName = filterJointName(_model.GetBodyName(i));
        _vectorDOF.append(filteredName, 0.0);
        _dofIndexToName.push_back(filteredName);
        _dofNameToIndex[filteredName] = i;
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
}
        
size_t Model::sizeDOF() const
{
    return _model.dof_count;
}
        
VectorLabel Model::getDOF() const
{
    return _vectorDOF;
}
void Model::setDOF(const VectorLabel& vect)
{
    _vectorDOF.mergeInter(vect);
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

    RBDLMath::VectorNd Q = RBDLMath::VectorNd::Zero(_model.dof_count);
    for (size_t i=0;i<_vectorDOF.size();i++) {
        Q(i) = _vectorDOF(_dofIndexToName.at(i));
    }

    RBDLMath::Vector3d ptSrc = point;
    RBDLMath::Vector3d ptBase = RBDL::CalcBodyToBaseCoordinates(
        _model, Q, srcFrameIndex, ptSrc);
    RBDLMath::Vector3d ptBody = RBDL::CalcBaseToBodyCoordinates(
        _model, Q, dstFrameIndex, ptBase);

    return ptBody;
}

const RBDL::Model& Model::getRBDLModel() const
{
    return _model;
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
 
}

