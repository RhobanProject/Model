#include "GrobanToePressureModel.hpp"

namespace Leph {

  std::map<std::string,std::string> GrobanToePressureModel::frameToPressure(
    {
      {"left_arch_tip" ,"LeftBase" },
      {"left_toe_tip"  ,"LeftToe"  },
      {"right_arch_tip","RightBase"},
      {"right_toe_tip" ,"RightToe" }
    });

  std::map<std::string,std::string> GrobanToePressureModel::pressureToFrame(
    {
      {"LeftBase" ,"left_ankle_roll" },
      {"LeftToe"  ,"left_toe_tip"  },
      {"RightBase","right_ankle_roll"},
      {"RightToe" ,"right_toe_tip" }
    });

    GrobanToePressureModel::GrobanToePressureModel(const std::string & filename,
                                                   const std::string & support)
    {
      for (const auto & supp : pressureToFrame) {
        models[supp.first] = HumanoidModelWithToe(filename, supp.second);
        pressureCOPs[supp.first] = Eigen::Vector3d(0,0,0);
        pressureWeights[supp.first] = 0;
      }
      currentSupport = support;
    }

    GrobanToePressureModel::~GrobanToePressureModel()
    {
    }

  const std::string & GrobanToePressureModel::getBasename() const
    {
      return currentSupport;
    }
  
    HumanoidModelWithToe& GrobanToePressureModel::get()
    {
      return models.at(currentSupport);
    }
  
    const HumanoidModelWithToe& GrobanToePressureModel::get() const
    {
      return models.at(currentSupport);
    }

    void GrobanToePressureModel::setPressure(const std::string & pressureName,
                                             double weight, double x, double y)
    {
      pressureWeights[pressureName] = weight;
      pressureCOPs[pressureName].x() = x;
      pressureCOPs[pressureName].y() = y;
    }

    void GrobanToePressureModel::updateBase()
    {
      std::string newSupport = "NONE";
      double maxWeight = 0;
      for (const auto & pressure : pressureWeights) {
        if (pressure.second > maxWeight) {
          maxWeight = pressure.second;
          newSupport = pressure.first;
        }
      }
      // Change support if required
      if (newSupport != currentSupport && newSupport != "NONE") {
        const std::string & newFrame = pressureToFrame[newSupport];
        Eigen::Vector3d posFrame = get().position(newFrame, "origin");
        double frameYaw = get().orientationYaw(newFrame, "origin");
        models[newSupport].importDOF(get());
        models[newSupport].setDOF("base_x", posFrame.x());
        models[newSupport].setDOF("base_y", posFrame.y());
        models[newSupport].setDOF("base_yaw", frameYaw);
        //TODO fix the problem due to using left_arch_tip and right_arch_tip
        if (newSupport.find("Base") != std::string::npos) {
          models[newSupport].setDOF("base_z", 0.0667);
        }
        else {
          models[newSupport].setDOF("base_z", 0.0);
        }
        currentSupport = newSupport;
      }

    }

    double GrobanToePressureModel::pressureWeight() const
    {
      double sum = 0;
      for (const auto & w : pressureWeights)
      {
        sum += w.second;
      }
      return sum;
    }

    double GrobanToePressureModel::pressureLeftRatio() const
    {
      double totalWeight = pressureWeight();
      double leftWeight = 0;
      for (const auto & wEntry : pressureWeights) {
        if (wEntry.first.find("Left") != std::string::npos) {
          leftWeight += wEntry.second;
        }
      }
      return leftWeight / totalWeight;
    }

    double GrobanToePressureModel::pressureRightRatio() const
    {
      return 1 - pressureLeftRatio();
    }

    double GrobanToePressureModel::pressureToeRatio() const
    {
      double totalWeight = pressureWeight();
      double toeWeight = 0;
      for (const auto & wEntry : pressureWeights) {
        if (wEntry.first.find("Toe") != std::string::npos) {
          toeWeight += wEntry.second;
        }
      }
      return toeWeight / totalWeight;
    }

    double GrobanToePressureModel::pressureBaseRatio() const
    {
      return 1 - pressureToeRatio();
    }
}
