#ifndef LEPH_SERIESUTILS_HPP
#define LEPH_SERIESUTILS_HPP

#include <string>
#include <vector>
#include "Types/MatrixLabel.hpp"
#include "Types/VectorLabel.hpp"
#include "TimeSeries/ModelSeries.hpp"
#include "TimeSeries/TimeSeries.hpp"
#include "Plot/Plot.hpp"

namespace Leph {

/**
 * Add to given Plot the two given 
 * TimeSeries name against each others
 */
void plotPhase(
    Leph::Plot& plot,
    Leph::ModelSeries& model, 
    const std::string& name1, 
    const std::vector<std::string>& names);
/**
 * Add to given Plot the given 
 * TimeSeries name against its future data
 */
void plotFuture(
    Leph::Plot& plot,
    Leph::ModelSeries& model, 
    const std::string& name);

/**
 * Predict and return a value with given 
 * inputs from given named regression model.
 */
double modelPredict(
    Leph::ModelSeries& model, 
    const std::string& name,
    const std::vector<double>& inputs);

/**
 * Compute the mean error and variance between
 * the two given time series.
 */
void seriesCompare(
    const Leph::TimeSeries& series1, 
    const Leph::TimeSeries& series2,
    double beginTime,
    double endTime,
    double& meanError,
    double& variance,
    int& count);

/**
 * Set up and configure the ModelSeries
 */
void initModelSeries(Leph::ModelSeries& model, 
    bool withMocapConcept,
    bool withDeltaRegression,
    bool withWalkRegression);

/**
 * Append to given ModelSeries data from
 * given VectorLabel at given time.
 * If withMocap is false, motion capture
 * data are not loaded
 */
void appendModelSeries(
    Leph::ModelSeries& model, 
    double time, 
    const Leph::VectorLabel& logs);

}

#endif

