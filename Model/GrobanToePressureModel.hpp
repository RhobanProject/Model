#pragma once

#include <string>
#include <Eigen/Dense>
#include "Model/HumanoidModelWithToe.hpp"

/**
 * TODO:
 * When multiple supports have a COP which is not at the border of the
 * sustentation area, errors could be splitted among degrees of freedom
 * in order to make both contact surface on the same plane
 */
namespace Leph {

  class GrobanToePressureModel
  {
  private:
    static std::map<std::string,std::string> frameToPressure;
    static std::map<std::string,std::string> pressureToFrame;
    
  public:
        
    /**
     * Initialization with given model type
     */
    GrobanToePressureModel(const std::string & filename,
                           const std::string & support);
    virtual ~GrobanToePressureModel();

    HumanoidModelWithToe& get();
    const HumanoidModelWithToe& get() const;

    const std::string & getBasename() const;

    /**
     * Set current pressure state for a given tag
     */
    void setPressure(const std::string & pressureName,
                     double weight, double x, double y);

    /**
     * Update base according to pressure data
     */
    void updateBase() ;

    /**
     * Return total weight in grams,
     * Foot weight ratio between 0 and 1.
     */
    double pressureWeight() const;
    double pressureLeftRatio() const;
    double pressureRightRatio() const;
    double pressureToeRatio() const;
    double pressureBaseRatio() const;

    /**
     * Compute and return the center of pressure in given frame, for specified
     * pressure plane (in meters)
     */
    Eigen::Vector3d centerOfPressure(const std::string& frame, const std::string & pressureName);

  private:

    /**
     * Only one model is active at a given moment, but several model may coexist
     * at the same time
     */
    std::string currentSupport;
    std::map<std::string,HumanoidModelWithToe> models;
    /**
     * weights are expressed in grams
     * COP are expressed in frame referential and unit is meter
     */
    std::map<std::string,Eigen::Vector3d> pressureCOPs;
    std::map<std::string,double> pressureWeights;
  };

}
