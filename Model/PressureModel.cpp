#include "Model/RBDLRootUpdate.h"
#include "Model/PressureModel.hpp"

namespace Leph {

  PressureModel::PressureModel(): Model()
  {
  }

  PressureModel::PressureModel(RBDL::Model & model) : Model(model)
  {
  }

  PressureModel::PressureModel(const std::string & filename) : Model(filename)
  {
  }
        
        
  PressureModel::~PressureModel()
  {
  }

  void PressureModel::updateGaugeList()
  {
    pressureValues.clear();
    for (const std::string& frameName : getFrames()) {
      if (frameName.find("gauge") != std::string::npos) {
        pressureValues[frameName] = 0;
      }
    }
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

