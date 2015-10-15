#include <stdexcept>
#include <cmath>
#include "Model/Model.hpp"
#include "Model/RBDLClosedLoop.h"

namespace Leph {

Model::Model() :
    _model(),
    _dofIndexToName(),
    _dofNameToIndex(),
    _dofs(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId()
{
}
        
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
    RBDL::Model model;
    if (!RBDL::Addons::URDFReadFromFile(
        filename.c_str(), &model, false)
    ) {
        throw std::runtime_error(
            "Model unable to load URDF file: " + filename);
    }

    //Parse and load RBDL model
    initializeModel(model);
}
        
Model::Model(RBDL::Model& model) :
    _model(),
    _dofIndexToName(),
    _dofNameToIndex(),
    _dofs(),
    _vectorDOF(),
    _frameIndexToName(),
    _frameNameToIndex(),
    _frameIndexToId()
{
    //Parse and load RBDL model
    initializeModel(model);
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
double Model::getDOF(size_t index) const
{
    if (index >= _dofIndexToName.size()) {
        throw std::logic_error("Model invalid DOF index");
    }
    return _dofs(index);
}
void Model::setDOF(const VectorLabel& vect, bool setBase)
{
    loadLabelToEigen(vect, _dofs, setBase);
}
void Model::setDOF(const std::string& name, double value)
{
    _dofs(_dofNameToIndex.at(name)) = value;
}
void Model::setDOF(size_t index, double value)
{
    if (index >= _dofIndexToName.size()) {
        throw std::logic_error("Model invalid DOF index");
    }
    _dofs(index) = value;
}

void Model::setDOFZeros()
{
    _dofs.setZero();
}
        
const std::string& Model::getDOFName(size_t index) const
{
    if (index >= _dofIndexToName.size()) {
        throw std::logic_error("Model invalid DOF index");
    }
    return _dofIndexToName.at(index);
}
size_t Model::getDOFIndex(const std::string& name) const
{
    return _dofNameToIndex.at(name);
}

const Eigen::VectorXd& Model::getDOFVect() const
{
    return _dofs;
}
void Model::setDOFVect(const Eigen::VectorXd& vect)
{
    if (vect.size() != _dofs.size()) {
        throw std::logic_error(
            "Model invalid DOF vector size");
    }
    _dofs = vect;
}
        
void Model::importDOF(Model& model)
{
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        _dofs(i) = model._dofs(
            model._dofNameToIndex.at(_dofIndexToName.at(i)));
    }
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
    if (srcFrameIndex == dstFrameIndex) {
        return point;
    }

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
        
double Model::orientationYaw(
    size_t srcFrameIndex,
    size_t dstFrameIndex)
{
    Eigen::Matrix3d rotation = 
        orientation(srcFrameIndex, dstFrameIndex);
    rotation.transposeInPlace();

    return atan2(rotation(1, 0), rotation(0, 0));
}
double Model::orientationYaw(
    const std::string& srcFrame,
    const std::string& dstFrame)
{
    Eigen::Matrix3d rotation = orientation(srcFrame, dstFrame);
    rotation.transposeInPlace();
    
    return atan2(rotation(1, 0), rotation(0, 0));
}
        
Eigen::MatrixXd Model::pointJacobian(
    const std::string& srcFrame,
    const Eigen::Vector3d& point)
{
    //Convert to body id
    size_t srcFrameIndex = getFrameIndex(srcFrame);
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    
    //Init matrix
    RBDLMath::MatrixNd G(6, _model.qdot_size);
    G.setZero();

    //Compute jacobian on given point
    CalcPointJacobian6D(_model, _dofs,
        srcFrameIndex, point, G, true);
        
    return G;
}

Eigen::VectorXd Model::pointVelocity(
    const std::string& srcFrame, 
    const Eigen::VectorXd& velocity,
    const Eigen::Vector3d& point)
{
    //Convert to body id
    size_t srcFrameIndex = getFrameIndex(srcFrame);
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    
    //Compute velocity
    return CalcPointVelocity6D(_model, _dofs, 
        velocity, srcFrameIndex, point, true);
}
Eigen::VectorXd Model::pointAcceleration(
    const std::string& srcFrame, 
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration,
    const Eigen::Vector3d& point)
{
    //Convert to body id
    size_t srcFrameIndex = getFrameIndex(srcFrame);
    srcFrameIndex = _frameIndexToId.at(srcFrameIndex);
    
    //Compute acceleration
    return CalcPointAcceleration6D(_model, _dofs, 
        velocity, acceleration, srcFrameIndex, point, true);
}
        
Eigen::Vector3d Model::centerOfMass(size_t frameIndex)
{
    double mass;
    RBDLMath::Vector3d com;
    RBDL::Utils::CalcCenterOfMass(_model, _dofs, _dofs, mass, com);

    return position(getFrameIndex("origin"), frameIndex, com);
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
        
Eigen::VectorXd Model::inverseDynamics(
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    RBDLMath::VectorNd QDot;
    RBDLMath::VectorNd QDDot;
    if (velocity.size() == 0) {
        QDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    } else {
        QDot = velocity;
    }
    if (acceleration.size() == 0) {
        QDDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (acceleration.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    } else {
        QDDot = acceleration;
    }

    RBDLMath::VectorNd tau(_model.dof_count);
    tau.setZero();
    RBDL::InverseDynamics(
        _model, _dofs, QDot, QDDot,
        tau, NULL);

    return tau;
}
VectorLabel Model::inverseDynamics(
    const VectorLabel& velocity,
    const VectorLabel& acceleration)
{
    //Declare Eigen Vector
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(_model.dof_count);
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(_model.dof_count);
    //Convertion from VectorLabel
    loadLabelToEigen(velocity, vel, false);
    loadLabelToEigen(acceleration, acc, false);
    //Given velocity and acceleration of
    //base pitch and roll are also used
    vel(Model::getDOFIndex("base_pitch")) = velocity("base_pitch");
    vel(Model::getDOFIndex("base_roll")) = velocity("base_roll");
    acc(Model::getDOFIndex("base_pitch")) = acceleration("base_pitch");
    acc(Model::getDOFIndex("base_roll")) = acceleration("base_roll");

    //Call implementation
    Eigen::VectorXd torques = inverseDynamics(vel, acc);

    //Back conversion
    VectorLabel vect = _vectorDOF;
    for (size_t i=0;i<vect.size();i++) {
        const std::string& label = vect.getLabel(i);
        vect(i) = torques(_dofNameToIndex.at(label));
    }

    return vect;
}

Eigen::VectorXd Model::inverseDynamicsClosedLoop(
    size_t fixedFrameIndex,
    bool useInfinityNorm, 
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    RBDLMath::VectorNd QDot;
    RBDLMath::VectorNd QDDot;
    if (velocity.size() == 0) {
        QDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (velocity.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid velocity vector size");
    } else {
        QDot = velocity;
    }
    if (acceleration.size() == 0) {
        QDDot = RBDLMath::VectorNd::Zero(_model.dof_count);
    } else if (acceleration.size() != _model.dof_count) {
        throw std::logic_error(
            "Model invalid acceleration vector size");
    } else {
        QDDot = acceleration;
    }

    unsigned int fixedFrameId = _frameIndexToId.at(fixedFrameIndex);
    return RBDLClosedLoopInverseDynamics(
        _model, _dofs, QDot, QDDot,
        fixedFrameId, useInfinityNorm);
}
Eigen::VectorXd Model::inverseDynamicsClosedLoop(
    const std::string& fixedFrameName,
    bool useInfinityNorm, 
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    return inverseDynamicsClosedLoop(
        getFrameIndex(fixedFrameName), 
        useInfinityNorm, 
        velocity, acceleration);
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

    return filtered;
}
        
std::string Model::filterFrameName(const std::string& name) const
{
    std::string filtered = name;
    size_t pos = filtered.rfind("_link");
    if (pos != std::string::npos) {
        filtered = filtered.substr(0, pos);
    } 
    
    return filtered;
}
        
void Model::initializeModel(RBDL::Model& model)
{
    //Assign RBDL model
    _model = model;
    //Build name-index joint mapping 
    //and VectorLabel structure
    for (size_t i=1;i<_model.mBodies.size();i++) {
        unsigned int virtualDepth = 0;
        std::string filteredName = filterJointName(
            getRBDLBodyName(i, virtualDepth));
        //Handle special case of 6 virtual bodies added by
        //the floating joint
        if (virtualDepth == 5) {
            addDOF(filteredName + "_x");
            addDOF(filteredName + "_y");
            addDOF(filteredName + "_z");
            addDOF(filteredName + "_yaw");
            addDOF(filteredName + "_pitch");
            addDOF(filteredName + "_roll");
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
        _vectorDOF(_dofIndexToName.at(i)) = _dofs(i);
    }
}

void Model::loadLabelToEigen(const VectorLabel& vect, 
    Eigen::VectorXd& dst, bool setBase)
{
    for (size_t i=0;i<vect.size();i++) {
        const std::string& label = vect.getLabel(i);
        if (
            _dofNameToIndex.count(label) != 0 &&
            (setBase || 
            label.find("base_") == std::string::npos)
        ) {
            dst(_dofNameToIndex.at(label)) = vect(i);
        }
    }
}
 
}

