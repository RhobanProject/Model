#include "Model/RBDLRootUpdate.h"
#include "Model/PressureModel.hpp"

namespace Leph {

  PressureModel::PressureModel(): Model(), ik(NULL)
  {
    init();
  }

  PressureModel::PressureModel(RBDL::Model & model) : Model(model), ik(NULL)
  {
    init();
  }

  PressureModel::PressureModel(const std::string & filename) : Model(filename), ik(NULL)
  {
    init();
  }
        
        
  PressureModel::~PressureModel()
  {
    if (ik != NULL) {
      delete(ik);
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
    for (const std::string& frameName : getFrames()) {
      if (frameName.find("gauge") != std::string::npos) {
        pressureValues[frameName] = 0;
      }
    }
  }

  void PressureModel::initIK()
  {
    //TODO
  }

  const std::map<std::string, double>& PressureModel::getPressureValues() const
  {
    return pressureValues;
  }


  void PressureModel::updatePressure(const Leph::VectorLabel& pressureData)
  {
    for (auto& pressureEntry : pressureValues) {
      pressureEntry.second = pressureData(pressureEntry.first);
    }
  }

  void PressureModel::updateBase()
  {
    //TODO
  }

}

