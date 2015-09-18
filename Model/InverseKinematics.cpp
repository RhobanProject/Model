#include <stdexcept>
#include <random>
#include "Model/InverseKinematics.hpp"

namespace Leph {

  // Apply the root squared on each element
  static Eigen::Vector3d rsCWise(const Eigen::Vector3d & v)
  {
    return Eigen::Vector3d(std::sqrt(v.x()),
                           std::sqrt(v.y()),
                           std::sqrt(v.z()));
  }


/**
 * Optimized computation of point jacobian
 * The function is copied from RBDL::CalcPointJacobian
 * (Kinematics.cc) and optimized to skipped the computation
 * of unused degree of freedom
 */
static void CustomCalcPointJacobian(
    RBDL::Model& model,
    const RBDLMath::VectorNd& Q,
    unsigned int body_id,
    const RBDLMath::Vector3d& point_position,
    RBDLMath::MatrixNd& fjac,
    size_t index, 
    const std::map<size_t, size_t>& globalIndexToSubset,
    const Eigen::Vector3d & weight)
{
    RBDLMath::SpatialTransform point_trans = 
        RBDLMath::SpatialTransform(RBDLMath::Matrix3d::Identity(), 
        RBDL::CalcBodyToBaseCoordinates(model, Q, body_id, point_position, false));
	
    unsigned int reference_body_id = body_id;
	
    if (model.IsFixedBodyId(body_id)) {
        unsigned int fbody_id = body_id - model.fixed_body_discriminator;
        reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    }

    unsigned int j = reference_body_id;

    while (j != 0) {
        unsigned int q_index = model.mJoints[j].q_index;
        if (globalIndexToSubset.count(q_index) != 0) {
            size_t subsetIndex = globalIndexToSubset.at(q_index);
            if (model.mJoints[j].mDoFCount == 3) {
                throw std::logic_error(
                    "InverseKinematics RBDL CalcPointJacobian not implemented");
            } else {
                fjac.block(index, subsetIndex, 3, 1) = point_trans.apply(
                    model.X_base[j].inverse().apply(model.S[j])
                  ).block(3,0,3,1).cwiseProduct(rsCWise(weight));
            }
        } 
        j = model.lambda[j];
    }
}
        
InverseKinematics::InverseKinematics(Model& model) :
    _model(&model),
    _subsetIndexToGlobal(),
    _globalIndexToSubset(),
    _dofs(),
    _allDofs(model._dofs),
    _lowerBounds(),
    _upperBounds(),
    _isLowerBounds(),
    _isUpperBounds(),
    _targetPositions(),
    _targetOrientations(),
    _targetScalars(),
    _isTargetCOM(false),
    _errorCOM(0.0),
    _targetCOM(),
    _errorSum(0.0)
{
}
        
void InverseKinematics::addDOF(const std::string& name)
{
    //Adding an association to the mapping
    size_t indexGlobal = _model->_dofNameToIndex.at(name);
    size_t indexSubset = _subsetIndexToGlobal.size();
    _subsetIndexToGlobal.push_back(indexGlobal);
    _lowerBounds.push_back(0.0);
    _upperBounds.push_back(0.0);
    _isLowerBounds.push_back(false);
    _isUpperBounds.push_back(false);
    _globalIndexToSubset[indexGlobal] = indexSubset;
    //Initial value
    _dofs.conservativeResize(indexSubset+1, Eigen::NoChange_t());
    _dofs(indexSubset) = _model->_dofs(indexGlobal);
}

void InverseKinematics::setLowerBound(const std::string& name, 
    double value)
{
    size_t indexGlobal = _model->_dofNameToIndex.at(name);
    if (_globalIndexToSubset.count(indexGlobal) == 0) {
        throw std::logic_error(
            "InverseKinematics invalid DOF name: '" + name + "'");
    }
    size_t indexSubset = _globalIndexToSubset.at(indexGlobal);

    _isLowerBounds[indexSubset] = true;
    _lowerBounds[indexSubset] = value;
}
void InverseKinematics::setUpperBound(const std::string& name, 
    double value)
{
    size_t indexGlobal = _model->_dofNameToIndex.at(name);
    if (_globalIndexToSubset.count(indexGlobal) == 0) {
        throw std::logic_error(
            "InverseKinematics invalid DOF name: '" + name + "'");
    }
    size_t indexSubset = _globalIndexToSubset.at(indexGlobal);

    _isUpperBounds[indexSubset] = true;
    _upperBounds[indexSubset] = value;
}
void InverseKinematics::clearLowerBound(const std::string& name)
{
    size_t indexGlobal = _model->_dofNameToIndex.at(name);
    if (_globalIndexToSubset.count(indexGlobal) == 0) {
        throw std::logic_error(
            "InverseKinematics invalid DOF name: '" + name + "'");
    }
    size_t indexSubset = _globalIndexToSubset.at(indexGlobal);

    _isLowerBounds[indexSubset] = false;
}
void InverseKinematics::clearUpperBound(const std::string& name)
{
    size_t indexGlobal = _model->_dofNameToIndex.at(name);
    if (_globalIndexToSubset.count(indexGlobal) == 0) {
        throw std::logic_error(
            "InverseKinematics invalid DOF name: '" + name + "'");
    }
    size_t indexSubset = _globalIndexToSubset.at(indexGlobal);

    _isUpperBounds[indexSubset] = false;
}
        
void InverseKinematics::addTargetPosition(
    const std::string& targetName,
    const std::string& srcFrame,
    const Eigen::Vector3d& point)
{
    if (_targetPositions.count(targetName) != 0) {
        throw std::logic_error(
            "InverseKinematics target already used");
    }

    //Convert frame name to body RBDL id
    size_t srcFrameIndex = _model->_frameNameToIndex.at(srcFrame);
    size_t srcFrameId = _model->_frameIndexToId.at(srcFrameIndex);

    //Add target to the container
    _targetPositions[targetName] = {
        targetName, 
        srcFrameId,
        point,
        Eigen::Vector3d::Zero(),
        0.0,
        Eigen::Vector3d::Constant(1.0)};

    //Default value
    targetPositionRef(targetName).target = _model->position(
        srcFrameIndex, _model->getFrameIndex("origin"), point);
}
        
void InverseKinematics::addTargetOrientation(
    const std::string& targetName,
    const std::string& srcFrame)
{
    if (_targetOrientations.count(targetName) != 0) {
        throw std::logic_error(
            "InverseKinematics target already used");
    }
    
    //Convert frame name to body RBDL id
    size_t srcFrameIndex = _model->_frameNameToIndex.at(srcFrame);
    size_t srcFrameId = _model->_frameIndexToId.at(srcFrameIndex);
    
    //Add target to the container
    _targetOrientations[targetName] = {
        targetName, 
        srcFrameId,
        Eigen::Matrix3d::Zero(),
        false,
        Eigen::Vector3d::Zero(),
        0.0,
        1.0};

    //Default value
    targetOrientationRef(targetName).target = _model->orientation(
        srcFrameIndex, _model->getFrameIndex("origin"));
}
        
void InverseKinematics::addTargetScalar(
    const std::string& targetName,
    const std::string& srcFrame,
    TargetAxis axis,
    const Eigen::Vector3d& point)
{
    if (_targetScalars.count(targetName) != 0) {
        throw std::logic_error(
            "InverseKinematics target already used");
    }
    
    //Convert frame name to body RBDL id
    size_t srcFrameIndex = _model->_frameNameToIndex.at(srcFrame);
    size_t srcFrameId = _model->_frameIndexToId.at(srcFrameIndex);
    
    //Add target to the container
    _targetScalars[targetName] = {
        targetName, 
        srcFrameId,
        Eigen::Vector3d::Zero(),
        axis,
        0.0,
        0.0,
        1.0};

    //Default value
    if (axis == AxisX) {
        targetScalarRef(targetName).target = _model->position(
            srcFrameIndex, _model->getFrameIndex("origin"), point).x();
    } else if (axis == AxisY) {
        targetScalarRef(targetName).target = _model->position(
            srcFrameIndex, _model->getFrameIndex("origin"), point).y();
    } else if (axis == AxisZ) {
        targetScalarRef(targetName).target = _model->position(
            srcFrameIndex, _model->getFrameIndex("origin"), point).z();
    }
}
        
void InverseKinematics::addTargetCOM()
{
    _isTargetCOM = true;
    _targetCOM = _model->centerOfMass("origin");
}

Eigen::Vector3d& InverseKinematics::targetPosition(
    const std::string& targetName)
{
    return targetPositionRef(targetName).target;
}
Eigen::Matrix3d& InverseKinematics::targetOrientation(
    const std::string& targetName)
{
    return targetOrientationRef(targetName).target;
}
double& InverseKinematics::targetScalar(
    const std::string& targetName)
{
    return targetScalarRef(targetName).target;
}
Eigen::Vector3d& InverseKinematics::targetCOM()
{
    if (!_isTargetCOM) {
        throw std::logic_error(
            "InverseKinematics COM target not enabled");
    }

    return _targetCOM;
}

double InverseKinematics::errorPosition(
    const std::string& targetName) const
{
    return targetPositionRef(targetName).error;
}
double InverseKinematics::errorOrientation(
    const std::string& targetName) const
{
    return targetOrientationRef(targetName).error;
}
double InverseKinematics::errorScalar(
    const std::string& targetName) const
{
    return targetScalarRef(targetName).error;
}
double InverseKinematics::errorCOM() const
{
    if (!_isTargetCOM) {
        throw std::logic_error(
            "InverseKinematics COM target not enabled");
    }

    return _errorCOM;
}

Eigen::Vector3d& InverseKinematics::weightPosition(
    const std::string& targetName)
{
    return targetPositionRef(targetName).weight;
}
double& InverseKinematics::weightOrientation(
    const std::string& targetName)
{
    return targetOrientationRef(targetName).weight;
}

double& InverseKinematics::weightScalar(
    const std::string& targetName)
{
    return targetScalarRef(targetName).weight;
}
Eigen::Vector3d& InverseKinematics::weightCOM()
{
    if (!_isTargetCOM) {
        throw std::logic_error(
            "InverseKinematics COM target not enabled");
    }

    return _weightCOM;
}
        
double InverseKinematics::errorSum() const
{
    return _errorSum;
}
        
void InverseKinematics::randomDOFNoise(double ampl)
{
    //Load current model DOF
    importDOF();
    //Random noise on DOF subset
    std::random_device rd;
    std::mt19937 generator(rd());
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        double pos = _dofs(i);
        double amplLow = ampl;
        double amplUp = ampl;
        if (_isLowerBounds[i]) {
            amplLow = std::min(ampl, pos-_lowerBounds[i]);
        }
        if (_isUpperBounds[i]) {
            amplUp = std::min(ampl, _upperBounds[i]-pos);
        }
        std::uniform_real_distribution<double> distribution(-amplLow, amplUp);
        _dofs(i) += distribution(generator);
    }
    //Save all computed DOF in model
    exportDOF();
}
        
void InverseKinematics::run(double tolerance, 
    unsigned int maxEvaluation)
{
    //Check if bodies which are orientation constrained are also
    //position constrained for target optimization
    for (const auto& pos : _targetPositions) {
        for (auto& orientation : _targetOrientations) {
            if (pos.second.bodyId == orientation.second.bodyId) {
                orientation.second.isPosTarget = true;
                orientation.second.posTarget = pos.second.target;
            }
        }
    }

    //Load current model DOF
    importDOF();
    //Bound state inside allowed bounds
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        if (_isLowerBounds[i] && _dofs(i) < _lowerBounds[i]) {
            _dofs(i) = _lowerBounds[i];
        }
        if (_isUpperBounds[i] && _dofs(i) > _upperBounds[i]) {
            _dofs(i) = _upperBounds[i];
        }
    }
    //Levenberg Marquardt initialization
    Eigen::LevenbergMarquardt<InverseKinematics> lm(*this);
    lm.setXtol(tolerance);
    lm.setFtol(tolerance);
    lm.setFactor(100.0);
    lm.setMaxfev(maxEvaluation);

    Eigen::LevenbergMarquardtSpace::Status st = lm.minimizeInit(_dofs);
    int count = 0;
    do {
        st = lm.minimizeOneStep(_dofs);
        count++;
    } while(st == Eigen::LevenbergMarquardtSpace::Running);
    
    //Save all computed DOF in model
    exportDOF();

    //Compute target error
    double squaredErrorSum = 0;
    size_t index = 0;
    // RootSquared is applied once more
    for (auto& target : _targetPositions) {
        Eigen::Vector3d errors = lm.fvec().segment(index, 3);
        Eigen::Vector3d rsWeight = rsCWise(target.second.weight);
        target.second.error = errors.cwiseProduct(rsWeight).stableNorm();
        index += 3;
        squaredErrorSum += target.second.error * target.second.error;
    }
    for (auto& target : _targetOrientations) {
        target.second.error = lm.fvec().segment(index, 6).stableNorm();
        target.second.error *= std::sqrt(target.second.weight);
        index += 6;
        squaredErrorSum += target.second.error * target.second.error;
    }
    for (auto& target : _targetScalars) {
        target.second.error = lm.fvec().segment(index, 1).stableNorm();
        target.second.error *= std::sqrt(target.second.weight);
        index += 1;
        squaredErrorSum += target.second.error * target.second.error;
    }
    if (_isTargetCOM) {
        Eigen::Vector3d errors = lm.fvec().segment(index, 3);
        Eigen::Vector3d rsWeight = rsCWise(_weightCOM);
        _errorCOM = errors.cwiseProduct(rsWeight).stableNorm();
        index += 3;
        squaredErrorSum += _errorCOM * _errorCOM;
    }
    _errorSum = std::sqrt(squaredErrorSum);
}
        
size_t InverseKinematics::sizeDOF() const
{
    return _subsetIndexToGlobal.size();
}
size_t InverseKinematics::sizeTarget() const
{
    return 
        3*_targetPositions.size() + 
        6*_targetOrientations.size() +
        1*_targetScalars.size() + 
        3*(size_t)_isTargetCOM;
}
        
size_t InverseKinematics::inputs() const
{
    return sizeDOF();
}
size_t InverseKinematics::values() const
{
    return (sizeDOF() <= sizeTarget()) ? 
        sizeTarget() : sizeDOF();
}

int InverseKinematics::operator()(const Eigen::VectorXd& dofs, 
    Eigen::VectorXd& fvec)
{
    //Update internal DOF complete vector
    updateAllDOF(dofs);
    //Update RBDL model
    RBDL::UpdateKinematicsCustom(_model->_model, &_allDofs, NULL, NULL);
    size_t index = 0;
    //Position targets
    for (const auto& target : _targetPositions) {
        //Convertion of constrainted point to target frame
        Eigen::Vector3d pt = RBDL::CalcBodyToBaseCoordinates(
            _model->_model, _allDofs, 
            target.second.bodyId, target.second.point, false);
        //Compute error
        fvec.segment(index, 3) = (pt - target.second.target).cwiseProduct(rsCWise(target.second.weight));
        index += 3;
    }
    //Orientation targets
    for (const auto& target : _targetOrientations) {
        //Real position of constrained body origin and two pseudo points
        Eigen::Vector3d realPt;
        //If available, apply target optimization (speed up 
        //convergence when real point target is available)
        if (target.second.isPosTarget) {
            realPt = target.second.posTarget;
        } else {
            realPt = RBDL::CalcBodyToBaseCoordinates(
                _model->_model, _allDofs, 
                target.second.bodyId, Eigen::Vector3d::Zero(), false);
        }
        Eigen::Vector3d realPtX = RBDL::CalcBodyToBaseCoordinates(
            _model->_model, _allDofs, 
            target.second.bodyId, Eigen::Vector3d(1.0, 0.0, 0.0), false);
        Eigen::Vector3d realPtY = RBDL::CalcBodyToBaseCoordinates(
            _model->_model, _allDofs, 
            target.second.bodyId, Eigen::Vector3d(0.0, 1.0, 0.0), false);
        //Compute target two points to force orientation
        Eigen::Vector3d targetPtX = target.second.target 
            * (realPt + Eigen::Vector3d(1.0, 0.0, 0.0));
        Eigen::Vector3d targetPtY = target.second.target 
            * (realPt + Eigen::Vector3d(0.0, 1.0, 0.0));
        //Compute error
        fvec.segment(index, 3) = (realPtX - targetPtX) * std::sqrt(target.second.weight);
        index += 3;
        fvec.segment(index, 3) = (realPtY - targetPtY) * std::sqrt(target.second.weight);
        index += 3;
    }
    //Scalar targets
    for (const auto& target : _targetScalars) {
        //Convertion of constrainted point to target frame
        Eigen::Vector3d pt = RBDL::CalcBodyToBaseCoordinates(
            _model->_model, _allDofs, 
            target.second.bodyId, target.second.point, false);
        //Compute error
        double rsWeight = std::sqrt(target.second.weight);
        if (target.second.axis == AxisX) {
          fvec(index) = (pt.x() - target.second.target) * rsWeight;
        } else if (target.second.axis == AxisY) {
          fvec(index) = (pt.y() - target.second.target) * rsWeight;
        } else if (target.second.axis == AxisZ) {
          fvec(index) = (pt.z() - target.second.target) * rsWeight;
        }
        index += 1;
    }
    //COM target
    if (_isTargetCOM) {
        //Compute current COM position
        double tmpMass;
        RBDLMath::Vector3d com;
        RBDL::Utils::CalcCenterOfMass(
            _model->_model, _allDofs, _allDofs, 
            tmpMass, com, NULL, NULL, false);
        //Compute error
        fvec.segment(index, 3) = (com - _targetCOM).cwiseProduct(rsCWise(weightCOM()));
        index += 3;
    }
    //Dummy errors values for eigen assert
    fvec.segment(index, values()-index).setZero();
    
    return 0;
}

int InverseKinematics::df(const Eigen::VectorXd& dofs, 
    Eigen::MatrixXd& fjac)
{
    //Reset to zero
    fjac.setZero();
    //Update internal DOF complete vector
    updateAllDOF(dofs);
    //Update RBDL model
    RBDL::UpdateKinematicsCustom(_model->_model, &_allDofs, NULL, NULL);
    size_t index = 0;
    //Position targets
    for (const auto& target : _targetPositions) {
        //Compute constrained point jacobian
        CustomCalcPointJacobian(
            _model->_model, _allDofs, target.second.bodyId, 
            target.second.point, fjac, index, _globalIndexToSubset, target.second.weight);
        index += 3;
    }
    //Orientation targets
    for (const auto& target : _targetOrientations) {
        //Compute constrained point X jacobian
        CustomCalcPointJacobian(
            _model->_model, _allDofs, target.second.bodyId, 
            Eigen::Vector3d(1.0, 0.0, 0.0), fjac, index, _globalIndexToSubset,
            Eigen::Vector3d::Constant(target.second.weight));
        index += 3;
        //Compute constrained point Y jacobian
        CustomCalcPointJacobian(
            _model->_model, _allDofs, target.second.bodyId, 
            Eigen::Vector3d(0.0, 1.0, 0.0), fjac, index, _globalIndexToSubset,
            Eigen::Vector3d::Constant(target.second.weight));
        index += 3;
    }
    //Scalar targets
    for (const auto& target : _targetScalars) {
        //Compute constrained point jacobian
        Eigen::MatrixXd tmpG = Eigen::MatrixXd::Zero(3, inputs());
        CustomCalcPointJacobian(
            _model->_model, _allDofs, target.second.bodyId, 
            target.second.point, tmpG, 0, _globalIndexToSubset,
            Eigen::Vector3d::Constant(target.second.weight));
        //Assign jacobian for used subset DOF
        if (target.second.axis == AxisX) {
            fjac.block(index, 0, 1, inputs()) = tmpG.block(0, 0, 1, inputs());
        } else if (target.second.axis == AxisY) {
            fjac.block(index, 0, 1, inputs()) = tmpG.block(1, 0, 1, inputs());
        } else if (target.second.axis == AxisZ) {
            fjac.block(index, 0, 1, inputs()) = tmpG.block(2, 0, 1, inputs());
        }
        index += 1;
    }
    //COM target
    if (_isTargetCOM) {
        //Compute COM jacobian
        comJacobian(fjac, index, weightCOM());
        index += 3;
    }

    return 0;
}
        
void InverseKinematics::gradientProjection(const Eigen::VectorXd& state,
    Eigen::VectorXd& gradient)
{
    Eigen::VectorXd nextState = state + gradient;
    for (size_t i=0;i<(size_t)state.size();i++) {
        if (_isLowerBounds[i] && nextState(i) < _lowerBounds[i]) {
            gradient(i) = _lowerBounds[i] - state(i);
        }
        if (_isUpperBounds[i] && nextState(i) > _upperBounds[i]) {
            gradient(i) = _upperBounds[i] - state(i);
        }
    }
}
        
const Eigen::VectorXd& InverseKinematics::getDOFSubset()
{
    importDOF();
    return _dofs;
}
        
void InverseKinematics::setDOFSubset(const Eigen::VectorXd& dofs)
{
    _dofs = dofs;
    exportDOF();
}

const std::vector<double>& InverseKinematics::getLowerBounds() const
{
    return _lowerBounds;
}
const std::vector<double>& InverseKinematics::getUpperBounds() const
{
    return _upperBounds;
}
const std::vector<bool>& InverseKinematics::getIsLowerBounds() const
{
    return _isLowerBounds;
}
const std::vector<bool>& InverseKinematics::getIsUpperBounds() const
{
    return _isUpperBounds;
}
        
void InverseKinematics::updateAllDOF(const Eigen::VectorXd& dofs)
{
    for (size_t i=0;i<(size_t)dofs.size();i++) {
        _allDofs(_subsetIndexToGlobal[i]) = dofs(i);
    }
}
        
void InverseKinematics::comJacobian(RBDLMath::MatrixNd& fjac, size_t index,
                                    const Eigen::Vector3d & weight)
{
    //Init com and mass
    double sumMass = 0.0;
    RBDLMath::MatrixNd tmpG(3, inputs());
    //Weighted average of jacobian on each body center of mass
    for (size_t i=1;i<_model->_model.mBodies.size();i++) {
        double mass = _model->_model.mBodies[i].mMass;
        const Eigen::Vector3d& center = 
            _model->_model.mBodies[i].mCenterOfMass;
        if (mass > 0.0) {
            tmpG.setZero();
            CustomCalcPointJacobian(
                _model->_model, _allDofs, i, 
                center, tmpG, 0, _globalIndexToSubset, weight);
            sumMass += mass;
            fjac.block(index, 0, 3, inputs()) += tmpG * mass;
        }
    }
    //Normalize the sum
    fjac.block(index, 0, 3, inputs()) *= (1.0/sumMass);
}
        
void InverseKinematics::importDOF()
{
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        _dofs(i) = _model->_dofs(_subsetIndexToGlobal.at(i));
    }
}
void InverseKinematics::exportDOF()
{
    for (size_t i=0;i<(size_t)_dofs.size();i++) {
        _model->_dofs(_subsetIndexToGlobal.at(i)) = _dofs(i);
    }
}

  const struct InverseKinematics::TargetPosition&
  InverseKinematics::targetPositionRef(const std::string & targetName) const
  {
    try {
      return _targetPositions.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetPosition: '"
                              + targetName + "'");
    }
  }
  const struct InverseKinematics::TargetOrientation&
  InverseKinematics::targetOrientationRef(const std::string & targetName) const
  {
    try {
      return _targetOrientations.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetOrientation: '"
                              + targetName + "'");
    }
  }
  const struct InverseKinematics::TargetScalar&
  InverseKinematics::targetScalarRef(const std::string & targetName) const
  {
    try {
      return _targetScalars.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetScalar: '"
                              + targetName + "'");
    }
  }
  struct InverseKinematics::TargetPosition&
  InverseKinematics::targetPositionRef(const std::string & targetName)
  {
    try {
      return _targetPositions.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetPosition: '"
                              + targetName + "'");
    }
  }
  struct InverseKinematics::TargetOrientation&
  InverseKinematics::targetOrientationRef(const std::string & targetName)
  {
    try {
      return _targetOrientations.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetOrientation: '"
                              + targetName + "'");
    }
  }
  struct InverseKinematics::TargetScalar&
  InverseKinematics::targetScalarRef(const std::string & targetName)
  {
    try {
      return _targetScalars.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetScalar: '"
                              + targetName + "'");
    }
  }


}

