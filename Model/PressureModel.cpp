#include "Model/RBDLRootUpdate.h"
#include "Model/PressureModel.hpp"

namespace Leph {

  PressureModel::PressureModel(): Model(), ik(NULL)
  {
    init();
  }

  PressureModel::PressureModel(const RBDL::Model & model) : Model(model), ik(NULL)
  {
    init();
  }

  PressureModel::PressureModel(const std::string & filename) : Model(filename), ik(NULL)
  {
    init();
  }

  PressureModel::PressureModel(const Leph::Model & model) : Model(model), ik(NULL)
  {
    init();
  }

  PressureModel::PressureModel(const Leph::PressureModel & other)
    : Model(other), ik(NULL), pressureValues(other.pressureValues),
      lastPressurePos(other.lastPressurePos), cop(other.cop), weight(other.weight)
  {
    init();
  }
        
        
  PressureModel::~PressureModel()
  {
    if (ik != NULL) {
      delete(ik);
      ik = NULL;
    }
  }

  void PressureModel::init()
  {
    initGaugeList();
    initIK();
  }

  void PressureModel::initGaugeList()
  {
    pressureValues.clear();
    lastPressurePos.clear();
    for (const std::string& frameName : getFrames()) {
      if (frameName.find("gauge") != std::string::npos) {
        pressureValues[frameName] = 0;
        lastPressurePos[frameName] = position(frameName, "origin");
      }
    }
  }

  void PressureModel::initIK()
  {
    if (ik != NULL) {
      delete(ik);
    }
    ik = new InverseKinematics(*this);
    // Adding all DOF from the model
    for (const std::string& dof : getDOFCategory("base")) {
      ik->addDOF(dof);
    }
    for (const std::string& dof : getDOFCategory("lowerBody")) {
      ik->addDOF(dof);
    }
    // Adding targets for every gauge frame
    for (const auto& pEntry : pressureValues) {
      ik->addTargetPosition(pEntry.first, pEntry.first);
    }
    ik->addTargetPosition("trunk", "trunk");
    // Adding target for every motor of the lower body
    for (const std::string & dof : getDOFCategory("lowerBody")){
      ik->addTargetDOF(dof, dof);
    }
  }

  void PressureModel::updateIK()
  {
    if (ik == NULL) {
      initIK();
    }
    // Weights
    double gaugeSlipWeight = 1;
    double gaugeZWeight = 20;
    double dofWeight = 10;
    // Tols
    double dofMaxError = 5 * M_PI / 180;
    // Threshold
    double gaugeThreshold = 200;
    Eigen::Vector3d gaugeWeight(gaugeSlipWeight, gaugeSlipWeight, gaugeZWeight);
    // Setting up DOF targets, weights and bounds
    for (const std::string & dof : getDOFCategory("lowerBody")){
      double readVal = getDOF(dof);
      ik->targetDOF(dof) = readVal;
      ik->weightDOF(dof) = dofWeight;
      ik->setLowerBound(dof, readVal - dofMaxError);
      ik->setUpperBound(dof, readVal + dofMaxError);
    }
    // Setting up Gauge targets and weights
    for (const auto& pEntry : pressureValues) {
      const std::string& gaugeName = pEntry.first;
      double gaugeVal = pEntry.second;
      //Target is always the same position as last known target;
      ik->targetPosition(gaugeName) = lastPressurePos.at(gaugeName);
      ik->targetPosition(gaugeName).z() = 0;
      //TODO do something else that binary choice here
      if (gaugeVal > gaugeThreshold) {
        ik->weightPosition(gaugeName) = gaugeWeight * (gaugeVal - gaugeThreshold) / gaugeThreshold;
      }
      // No cost if no pressure on gauge (ideally z < 0 should have a cost)
      else {
        ik->weightPosition(gaugeName) = Eigen::Vector3d::Zero();
      }
      // If target was not grounded at last step, we not to ignore x and y
      // constraint, otherwise the projection will produce a huge error and the
      // grounded gauges of the same feet will be moved
      if (abs(lastPressurePos.at(gaugeName).z()) > 0.001) {
        ik->weightPosition(gaugeName).x() = 0;
        ik->weightPosition(gaugeName).y() = 0;
      }
        
    }
    // Setting a very low weight on trunk condition to ensure there is enough
    // constraints on the system.
    ik->targetPosition("trunk") = lastTrunkPos;
    ik->weightPosition("trunk") = Eigen::Vector3d::Constant(0.01);
  }

  const std::map<std::string, double>& PressureModel::getPressureValues() const
  {
    return pressureValues;
  }

  double PressureModel::getTotalWeight() const
  {
    return weight;
  }

  Eigen::Vector3d PressureModel::getCOP(const std::string& frameName)
  {
    return position("origin", frameName, cop);
  }



  void PressureModel::updatePressure(const Leph::VectorLabel& pressureData)
  {
    for (auto& pressureEntry : pressureValues) {
      try {
        pressureEntry.second = pressureData(pressureEntry.first);
      }
      catch(const std::logic_error& exc) {
        throw std::runtime_error("PressureModel::updatePressure: missing data, cannot find pressure: '" + pressureEntry.first + "'"); 
      }
    }
  }

  void PressureModel::updatePressurePos()
  {
    for (auto& pressureEntry : lastPressurePos) {
      pressureEntry.second = position(pressureEntry.first, "origin");
    }
  }

  void PressureModel::updateBase()
  {
    updateIK();
    //ik->randomDOFNoise();
    ik->run(0.0001, 100);
    updatePressurePos();
    updateCOP();
    lastTrunkPos = position("trunk","origin");

//    std::cout << "TARGETS:" << std::endl << ik->getNamedTargets();
//    std::cout << "MARGINS:" << std::endl << ik->getNamedDOFMargins();
//    std::cout << "ERRORS:" << std::endl << ik->getNamedErrors();
  }

  void PressureModel::updateCOP()
  {
    double totalWeight = 0;
    Eigen::Vector3d totalCOP = Eigen::Vector3d::Zero();
    for (const auto& pEntry : pressureValues) {
      totalCOP += pEntry.second * lastPressurePos.at(pEntry.first);
      totalWeight += pEntry.second;      
    }
    weight = totalWeight;
    if (weight != 0) {
      cop = totalCOP / weight;
    }
    else {
      cop = Eigen::Vector3d::Zero();
    }
  }

}

