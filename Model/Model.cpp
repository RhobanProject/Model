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
        
VectorLabel Model::getDOF() const
{
    return _vectorDOF;
}
void Model::setDOF(const VectorLabel& vect)
{
    _vectorDOF.mergeInter(vect);
}
void Model::setDOF(const std::string& name, double value)
{
    _vectorDOF(name) = value;
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

    //Rebuild degree of freedom eigen vector
    RBDLMath::VectorNd Q = buildDOFVector();

    //Compute transformation from body1 to base and base to body2
    RBDLMath::Vector3d ptBase = RBDL::CalcBodyToBaseCoordinates(
        _model, Q, srcFrameIndex, point);
    RBDLMath::Vector3d ptBody = RBDL::CalcBaseToBodyCoordinates(
        _model, Q, dstFrameIndex, ptBase);

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
    
    //Rebuild degree of freedom eigen vector
    RBDLMath::VectorNd Q = buildDOFVector();
            
    RBDLMath::Matrix3d transform1 = CalcBodyWorldOrientation(
        _model, Q, srcFrameIndex);
    RBDLMath::Matrix3d transform2 = CalcBodyWorldOrientation(
        _model, Q, dstFrameIndex);

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
    //Rebuild degree of freedom eigen vector
    RBDLMath::VectorNd Q = buildDOFVector();

    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(_model, Q, Q, mass, com);

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
    _dofNameToIndex[name] = _dofNameToIndex.size();
    _dofIndexToName.push_back(name);
}
        
RBDLMath::VectorNd Model::buildDOFVector() const
{
    RBDLMath::VectorNd Q = RBDLMath::VectorNd::Zero(_model.dof_count);
    for (size_t i=0;i<_vectorDOF.size();i++) {
        const std::string& name = _dofIndexToName.at(i);
        //Assign angular value
        Q(i) = _vectorDOF(_dofIndexToName.at(i));
        //No convertion to radian in case of translation
        //(floating joint)
        if (
            name.find(" Tx") == std::string::npos &&
            name.find(" Ty") == std::string::npos &&
            name.find(" Tz") == std::string::npos
        ) {
            Q(i) = M_PI*Q(i)/180.0;
        } 
    }

    return Q;
}
 
}

