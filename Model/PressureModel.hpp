#pragma once

#include "Model/Model.hpp"
#include "Model/InverseKinematics.hpp"

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

    PressureModel(const RBDL::Model & model);

    PressureModel(const Leph::Model & model);

    // IK of the built model is not initialized, but set to NULL
    PressureModel(const PressureModel & model);

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

    double getTotalWeight() const;
    Eigen::Vector3d getCOP(const std::string& frameName);

  protected:
    InverseKinematics * ik;
    std::map<std::string, double> pressureValues;
    // It is necessary to store last pressure pos
    std::map<std::string, Eigen::Vector3d> lastPressurePos;
    // Also storing trunk position in order to ensure there is enough
    // constraint to avoid teleportation
    Eigen::Vector3d lastTrunkPos;


    //Position of the center of pressure in the base frame [m]
    Eigen::Vector3d cop;
    //Total weight at last update [g]
    double weight;

  private:
    /**
     * Ensure that gaugeList corresponds to the model and that
     * the ik set with the appropriate DOF and targets
     * Should be called if rbdlModel is updated
     */
    void init();
    /**
     * initGaugeList should be called prior to initIK
     */
    void initGaugeList();
    void initIK();

    /**
     * Set targets of IK and their weight according to received data
     * also set the bounds for DOF
     * It is necessary to update the model with read values before using it
     */
    void updateIK();
    /**
     * store current pressure position for next update
     */
    void updatePressurePos();
    /**
     * compute and store the CoP corresponding to the informations given by the
     * sensors and by the 
     */
    void updateCOP();
  };

}
