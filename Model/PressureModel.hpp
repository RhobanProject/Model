#pragma once

#include "Model/Model.hpp"

namespace Leph {

/**
 * PressureModel
 *
 * Inherit Model and implements possibilities to:
 * - Set the values of the gauges
 * - Update the base using the most recent gauges information received
 * - Retrieve the position of the Center of Pressure inside a given frame
 */
  class PressureModel : public Model
  {
  public:
    PressureModel();

    PressureModel(RBDL::Model & model);

    PressureModel(const std::string& filename);

    /**
     * Virtual destructor
     */
    virtual ~PressureModel();

    // Data for all pressure sensors should be sent
    void updatePressure(const Leph::VectorLabel& pressureData);

    // Update the base according to pressure informations
    void updateBase();

    const std::map<std::string, double>& getPressureValues() const;

  private:
    std::map<std::string, double> pressureValues;

    void updateGaugeList();
  };

}
