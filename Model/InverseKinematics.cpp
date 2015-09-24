#include <stdexcept>
#include <random>
#include "Model/InverseKinematics.hpp"

namespace Leph {

  // Apply the root squared on each element
  // All positions target are now treated axis by axis, so function is unused
  //static Eigen::Vector3d rsCWise(const Eigen::Vector3d & v)
  //{
  //  return Eigen::Vector3d(std::sqrt(v.x()),
  //                         std::sqrt(v.y()),
  //                         std::sqrt(v.z()));
  //}


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
    RBDLMath::SpatialTransform targetFromBase = 
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
              // Model.X_base[j] = Spatial Transformation from base to body j
              RBDLMath::SpatialTransform baseFromBodyJ = model.X_base[j].inverse();
              RBDLMath::SpatialVector moveInBase = baseFromBodyJ.apply(model.S[j]);
              const auto& pointJac = targetFromBase.apply(moveInBase);
              size_t nbLinesWritten = 0;
              for (size_t axis = 0; axis < 3; axis++){
                if (weight(axis) != 0) {
                  fjac(index + nbLinesWritten, subsetIndex) = pointJac(3 + axis, 0) * std::sqrt(weight(axis));
                  nbLinesWritten++;
                }
              }
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

  VectorLabel InverseKinematics::getNamedDOFSubset()
  {
    VectorLabel result;
    for (size_t dofID = 0; dofID < inputs(); dofID++) {
      std::string name = _model->getDOFName(_subsetIndexToGlobal[dofID]);
      result.append(name, _dofs[dofID]);
    }
    return result;
  }

  VectorLabel InverseKinematics::getNamedTargets()
  {
    static const std::vector<std::string> axisName = {"x", "y", "z"};
    VectorLabel result; 
    for (const auto& entry : _targetPositions) {
      for (size_t axis = 0; axis < 3; axis ++) {
        if (entry.second.weight(axis) != 0) {
          result.append("pos:" + entry.first + ":" + axisName[axis],
                        entry.second.target(axis));
        }
      }
    }
    for (const auto& entry : _targetOrientations) {
      if (entry.second.weight != 0) {
        for (size_t axis1 = 0; axis1 < 2; axis1 ++) {
          for (size_t axis2 = 0; axis2 < 3; axis2 ++) {
            result.append("dir:" + entry.first + ":" + axisName[axis1] + axisName[axis2],
                          entry.second.target(axis1,axis2));
          }
        }
      }
    }
    for (const auto& entry : _targetDOFs) {
      if (entry.second.weight != 0) {
        result.append("dof:" + entry.first, entry.second.target);
      }
    }
    if (_isTargetCOM) {
      for (size_t axis = 0; axis < 3; axis ++) {
        if (_weightCOM(axis) != 0) {
          result.append("com:" + axisName[axis],
                        _targetCOM(axis));
        }
      }
    }
    return result;
  }

  VectorLabel InverseKinematics::getNamedErrors()
  {
    VectorLabel result; 
    for (const auto& entry : _targetPositions) {
      result.append("pos:" + entry.first, entry.second.error);
    }
    for (const auto& entry : _targetOrientations) {
      result.append("dir:" + entry.first, entry.second.error);
    }
    for (const auto& entry : _targetDOFs) {
      result.append("dof:" + entry.first, entry.second.error);
    }
    if (_isTargetCOM) {
      result.append("com", _errorCOM);
    }
    return result;
  }

  VectorLabel InverseKinematics::getNamedWeights()
  {
    static const std::vector<std::string> axisName = {"x", "y", "z"};
    VectorLabel result; 
    for (const auto& entry : _targetPositions) {
      for (size_t axis = 0; axis < 3; axis ++) {
        if (entry.second.weight(axis) != 0) {
          result.append("pos:" + entry.first + ":" + axisName[axis],
                        entry.second.weight(axis));
        }
      }
    }
    for (const auto& entry : _targetOrientations) {
      if (entry.second.weight != 0) {
        result.append("dir:" + entry.first, entry.second.weight);
      }
    }
    for (const auto& entry : _targetDOFs) {
      if (entry.second.weight != 0) {
        result.append("dof:" + entry.first, entry.second.weight);
      }
    }
    if (_isTargetCOM) {
      for (size_t axis = 0; axis < 3; axis ++) {
        if (_weightCOM(axis) != 0) {
          result.append("com:" + axisName[axis],
                        _weightCOM(axis));
        }
      }
    }
    return result;
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
    size_t srcFrameIndex = _model->getFrameIndex(srcFrame);
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
        
void InverseKinematics::addTargetDOF(const std::string& targetName,
                                     const std::string& dofName)
{
    if (_targetDOFs.count(targetName) != 0) {
        throw std::logic_error(
            "InverseKinematics target already used");
    }

    size_t globalIndex = _model->getDOFIndex(dofName);
    size_t subsetIndex;
    try {
      subsetIndex = _globalIndexToSubset.at(globalIndex);
    }
    catch (const std::out_of_range& exc) {
      throw std::out_of_range("InverseKinematics: Cannot set '" + dofName
                              + "' as a targetDOF, use addDOF(...) before");
    }

    //Add target to the container
    _targetDOFs[targetName] = {
      targetName,
      subsetIndex, 
      0.0,
      0.0,
      1.0};
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

double& InverseKinematics::targetDOF(
    const std::string& targetName)
{
    return targetDOFRef(targetName).target;
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
double InverseKinematics::errorDOF(
    const std::string& targetName) const
{
    return targetDOFRef(targetName).error;
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
double& InverseKinematics::weightDOF(
    const std::string& targetName)
{
    return targetDOFRef(targetName).weight;
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
      double targetSquaredError = 0;
      for (size_t axis = 0; axis < 3; axis ++) {
        double axisWeight = target.second.weight(axis);
        if (axisWeight != 0) {
          double error = lm.fvec()(index);
          targetSquaredError += error * error * axisWeight;
          index++;
        }
      }
      target.second.error = std::sqrt(targetSquaredError);
      squaredErrorSum += targetSquaredError;
    }
    for (auto& target : _targetOrientations) {
        auto errors = lm.fvec().segment(index, 6);
        double rsWeight = std::sqrt(target.second.weight);
        target.second.error = (errors * rsWeight).stableNorm();
        index += 6;
        squaredErrorSum += target.second.error * target.second.error;
    }
    for (auto& target : _targetDOFs) {
        target.second.error = lm.fvec().segment(index, 1).stableNorm();
        target.second.error *= std::sqrt(target.second.weight);
        index += 1;
        squaredErrorSum += target.second.error * target.second.error;
    }
    if (_isTargetCOM) {
      double comSquaredError = 0;
      for (size_t axis = 0; axis < 3; axis ++) {
        double axisWeight = _weightCOM(axis);
        if (axisWeight != 0) {
          double error = lm.fvec()(index);
          comSquaredError += error * error * axisWeight;
          index++;
        }
      }
      _errorCOM = std::sqrt(comSquaredError);
      squaredErrorSum += comSquaredError;
    }
    _errorSum = std::sqrt(squaredErrorSum);
}
        
size_t InverseKinematics::sizeDOF() const
{
    return _subsetIndexToGlobal.size();
}
size_t InverseKinematics::sizeTarget() const
{
  size_t nbTargets = 0;
  for (auto& target : _targetPositions) {
    for (size_t axis = 0; axis < 3; axis ++) {
      if (target.second.weight(axis) != 0) {
        nbTargets++;
      }
    }
  }
  for (auto& target : _targetOrientations) {
    if (target.second.weight != 0) {
      nbTargets += 6;
    }
  }
  for (auto& target : _targetDOFs) {
    if (target.second.weight != 0) {
      nbTargets++;
    }
  }
  if (_isTargetCOM) {
    for (size_t axis = 0; axis < 3; axis ++) {
      if (_weightCOM(axis) != 0) {
        nbTargets++;
      }
    }
  }
  return nbTargets;
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
        bool uselessTarget = true;
        for (size_t axis = 0; axis < 3; axis++) {
          if (target.second.weight(axis) != 0) {
            uselessTarget = false;
            break;
          }
        }
        if (uselessTarget) { continue; }
        //Convertion of constrainted point to target frame
        Eigen::Vector3d pt = RBDL::CalcBodyToBaseCoordinates(
            _model->_model, _allDofs, 
            target.second.bodyId, target.second.point, false);
        //Compute error
        Eigen::Vector3d error = pt - target.second.target;
        for (size_t axis = 0; axis < 3; axis++) {
          double axisWeight = target.second.weight(axis);
          if (axisWeight != 0) {
            fvec(index) = error(axis) * std::sqrt(axisWeight);
            index++;
          }
        }
    }
    //Orientation targets
    for (const auto& target : _targetOrientations) {
        // Avoid spending calculations time for targets with 0 weight
        if (target.second.weight == 0) { continue; }
        //Real position of constrained body origin and two pseudo points
        Eigen::Vector3d realPt;
        // Disabled:
        //  If available, apply target optimization (speed up 
        //  convergence when real point target is available)
        //- Does not work at all if target is not reachable!!!!
        if (false && target.second.isPosTarget) {
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
    //DOF targets
    for (const auto& target : _targetDOFs) {
      // Avoid spending calculations time for targets with 0 weight
      if (target.second.weight == 0) { continue; }
      size_t globalIndex = _subsetIndexToGlobal.at(target.second.subsetIndex);
      double val = _allDofs(globalIndex);
      //Compute error
      double rsWeight = std::sqrt(target.second.weight);
      fvec(index) = (val - target.second.target) * rsWeight;
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
        Eigen::Vector3d error = com - _targetCOM;
        for (size_t axis = 0; axis < 3; axis++) {
          double axisWeight = _weightCOM(axis);
          if (axisWeight != 0) {
            fvec(index) = error(axis) * std::sqrt(axisWeight);
            index++;
          }
        }
    }
    //Dummy errors values for eigen assert
    fvec.segment(index, values()-index).setZero();

    //std::cout << "fvec:" << std::endl << fvec << std::endl;
    
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
        bool uselessTarget = true;
        for (size_t axis = 0; axis < 3; axis++) {
          if (target.second.weight(axis) != 0) {
            uselessTarget = false;
            break;
          }
        }
        if (uselessTarget) { continue; }
        //Compute constrained point jacobian
        CustomCalcPointJacobian(
            _model->_model, _allDofs, target.second.bodyId, 
            target.second.point, fjac, index, _globalIndexToSubset, target.second.weight);
        for (size_t axis = 0; axis < 3; axis++) {
          if (target.second.weight(axis) != 0) {
            index++;
          }
        }
    }
    //Orientation targets
    for (const auto& target : _targetOrientations) {
        // Avoid spending calculations time for targets with 0 weight
        if (target.second.weight == 0) { continue; }
        // A huge fakeDistance is used in order to reduce impact of 'translations' on the jacobian
        double fakeDistance = std::pow(10,20);
        Eigen::MatrixXd tmpG = Eigen::MatrixXd::Zero(3, inputs());;
        //Compute constrained point X jacobian
        CustomCalcPointJacobian(
            _model->_model, _allDofs, target.second.bodyId, 
            Eigen::Vector3d(fakeDistance, 0.0, 0.0), tmpG, 0, _globalIndexToSubset,
            Eigen::Vector3d::Constant(target.second.weight));
        fjac.block(index, 0, 3, inputs()) = tmpG.block(0, 0, 3, inputs()) / fakeDistance;
        index += 3;
        //Compute constrained point Y jacobian
        CustomCalcPointJacobian(
            _model->_model, _allDofs, target.second.bodyId, 
            Eigen::Vector3d(0.0, fakeDistance, 0.0), tmpG, 0, _globalIndexToSubset,
            Eigen::Vector3d::Constant(target.second.weight));
        fjac.block(index, 0, 3, inputs()) = tmpG.block(0, 0, 3, inputs()) / fakeDistance;
        index += 3;
    }
    //DOF targets
    for (const auto& target : _targetDOFs) {
      // Avoid spending calculations time for targets with 0 weight
      if (target.second.weight == 0) { continue; }
      Eigen::MatrixXd tmpG = Eigen::MatrixXd::Zero(1, inputs());
      tmpG(0, target.second.subsetIndex) = std::sqrt(target.second.weight);
      fjac.block(index, 0, 1, inputs()) = tmpG.block(0, 0, 1, inputs());
      index += 1;
    }
    //COM target
    if (_isTargetCOM) {
        //Compute COM jacobian
        comJacobian(fjac, index, weightCOM());
        for (size_t axis = 0; axis < 3; axis++) {
          if (_weightCOM(axis != 0)) {
            index++;
          }
        }
    }

    //std::cout << "Jacobian:" << std::endl << fjac << std::endl;

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
    size_t nbAxisUsed = 0;
    for (size_t axis = 0; axis < 3; axis++) {
      if (weight(axis) != 0) {
        nbAxisUsed++;
      }
    }
    if (nbAxisUsed == 0) {
      throw std::runtime_error("IK:comJacobian: trying to compute a jacobian with 0 axis used");
    }

    RBDLMath::MatrixNd tmpG(nbAxisUsed, inputs());
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
            fjac.block(index, 0, nbAxisUsed, inputs()) += tmpG * mass;
        }
    }
    //Normalize the sum
    fjac.block(index, 0, nbAxisUsed, inputs()) *= (1.0/sumMass);
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
  const struct InverseKinematics::TargetDOF&
  InverseKinematics::targetDOFRef(const std::string & targetName) const
  {
    try {
      return _targetDOFs.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetDOF: '"
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
  struct InverseKinematics::TargetDOF&
  InverseKinematics::targetDOFRef(const std::string & targetName)
  {
    try {
      return _targetDOFs.at(targetName);
    }
    catch (const std::out_of_range & exc) {
      throw std::out_of_range("InverseKinematics: cannot find targetDOF: '"
                              + targetName + "'");
    }
  }


}

