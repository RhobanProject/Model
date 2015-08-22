#ifndef LEPH_REGRESSION_HPP
#define LEPH_REGRESSION_HPP

#include <vector>
#include <stdexcept>
#include <cmath>
#include <libcmaes/cmaes.h>
#include "TimeSeries/TimeSeries.hpp"
#include "TimeSeries/Optimizable.hpp"

namespace Leph {

/**
 * Regression
 *
 * Base class for regression model learning
 * from input time series to target output
 * time series.
 */
class Regression : public Optimizable
{
    public:

        /**
         * Structure for regression input.
         * Input is defined as a time series and a time 
         * offset in past used to get the input value
         */
        struct Input {
            const TimeSeries* series;
            double deltaTime;
        };

        /**
         * Initialization
         */
        Regression() :
            Optimizable(),
            _inputSeries(),
            _outputSeries(nullptr)
        {
        }

        /**
         * Virtual desctructor
         */
        virtual ~Regression()
        {
        }
        
        /**
         * Return the number of 
         * registered inputs time series 
         */
        inline size_t inputSize() const
        {
            return _inputSeries.size();
        }

        /**
         * Direct access to internal
         * input and output time series pointer.
         */
        inline const Input& getInput(size_t index) const
        {
            if (index >= _inputSeries.size()) {
                throw std::logic_error("Regression invalid index");
            }

            return _inputSeries[index];
        }
        inline const TimeSeries* getOutput() const
        {
            return _outputSeries;
        }

        /**
         * Append a new regression input with given 
         * TimeSeries pointer and time lag.
         * Time lag could be either an float time offset or
         * an index offset
         */
        inline void addInput(const TimeSeries* ptr, double deltaTime = 0.0)
        {
            if (ptr == nullptr) {
                throw std::logic_error("Regression null input pointer");
            }

            //Append input
            _inputSeries.push_back({ptr, deltaTime});
            //Reset meta parameters
            Optimizable::resetParameters();
            //Call handler
            onAddInput();
        }

        /**
         * Set output time series pointer
         * at given index.
         * (pointer are intialized to nullptr and
         * must be defined)
         */
        inline void setOutput(const TimeSeries* ptr)
        {
            if (ptr == nullptr) {
                throw std::logic_error("Regression null output pointer");
            }
            
            _outputSeries = ptr;
        }

        /**
         * Try to use input and output series at given time point
         * to update the regression model.
         * True is returned if update is successful.
         * False is returnded if required data are not available.
         */
        virtual bool learn(double time) = 0;

        /**
         * Try to use input series at given time point
         * to predict output data. If required data points
         * are not available, std::runtime_error is thrown.
         */
        virtual double predict(double time) const = 0;

        /**
         * Reset learned model to empty
         */
        virtual void resetRegression() = 0;

        /**
         * Learn ranged points between beginTime and endTime from
         * inputs and outputs series. A new learning point is
         * created each time all inputs are new.
         * Return false if no model update was done due to
         * inputs not available
         */
        inline bool rangeLearn(double beginTime, double endTime)
        {
            if (inputSize() == 0 || _outputSeries == nullptr) {
                throw std::logic_error("Regression not initialized");
            }

            //Find real min and max time bound
            double timeMin = beginTime;
            double timeMax = endTime;
            for (size_t i=0;i<_inputSeries.size();i++) {
                if (_inputSeries[i].series->timeMin() > timeMin) {
                    timeMin = _inputSeries[i].series->timeMin();
                }
                if (_inputSeries[i].series->timeMax() < timeMax) {
                    timeMax = _inputSeries[i].series->timeMax();
                }
            }

            //Go through all ranged input values
            bool isUpdated = false;
            double currentTime = timeMin;
            while (currentTime < timeMax) {
                //Find the time associated with 
                //all inputs updated at least once
                currentTime += TIME_EPSILON;
                double nextTime = std::numeric_limits<double>::quiet_NaN();
                for (size_t i=0;i<_inputSeries.size();i++) {
                    size_t indexLow = _inputSeries[i].series
                        ->getLowerIndex(currentTime);
                    double timeLow = _inputSeries[i].series
                        ->at(indexLow).time;
                    if (std::isnan(nextTime) || timeLow > nextTime) {
                        nextTime = timeLow;
                    }
                }
                if (std::isnan(nextTime)) {
                    throw std::logic_error("Regression error nan");
                }
                //Try to learn at this time if all inputs
                //are available
                if (learn(nextTime) == true) {
                    isUpdated = true;
                }
                //Go to next point
                currentTime = nextTime;
            }

            return isUpdated;
        }

        /**
         * Compute and return the prediction root mean 
         * squared error on ranged points between beginTime and
         * endTime.
         * If no prediction can be made, -1.0 is returned.
         */
        inline double rangeMSE(double beginTime, double endTime) const
        {
            if (inputSize() == 0 || _outputSeries == nullptr) {
                throw std::logic_error("Regression not initialized");
            }

            //Find real min and max time bound
            double timeMin = beginTime;
            double timeMax = endTime;
            for (size_t i=0;i<_inputSeries.size();i++) {
                if (_inputSeries[i].series->timeMin() > timeMin) {
                    timeMin = _inputSeries[i].series->timeMin();
                }
                if (_inputSeries[i].series->timeMax() < timeMax) {
                    timeMax = _inputSeries[i].series->timeMax();
                }
            }

            //Go through all ranged input values
            double sumSquareError = 0.0;
            int countPoint = 0;
            double currentTime = timeMin;
            while (currentTime < timeMax) {
                //Find the time associated with 
                //all inputs updated at least once
                currentTime += TIME_EPSILON;
                double nextTime = std::numeric_limits<double>::quiet_NaN();
                for (size_t i=0;i<_inputSeries.size();i++) {
                    size_t indexLow = _inputSeries[i].series
                        ->getLowerIndex(currentTime);
                    double timeLow = _inputSeries[i].series
                        ->at(indexLow).time;
                    if (std::isnan(nextTime) || timeLow > nextTime) {
                        nextTime = timeLow;
                    }
                }
                if (std::isnan(nextTime)) {
                    throw std::logic_error("Regression error nan");
                }
                //Try to predic at this time if all inputs
                //are available
                try {
                    double yp = predict(nextTime);
                    //Compute prediction error
                    double error = yp - _outputSeries->get(nextTime);
                    sumSquareError = pow(error, 2);
                    countPoint++;
                } catch (const std::runtime_error& e) {
                }
                //Go to next point
                currentTime = nextTime;
            }

            if (countPoint > 0) {
                return sqrt(sumSquareError/(double)countPoint);
            } else {
                return -1.0;
            }
        }

        /**
         * Optimize regression meta parameters using CMAES.
         * Points between beginTimeLearn and endTimeLearn are used
         * for learning model and points between beginTimeTest and
         * endTimeTest for test.
         * Maximum number of iterations is given.
         */
        inline void optimizeParameters(
            double beginTimeLearn, double endTimeLearn,
            double beginTimeTest, double endTimeTest,
            unsigned int maxIteration, bool isQuiet)
        {
            //Starting point conversion to vector
            std::vector<double> x0(parameterSize());
            for (size_t i=0;i<parameterSize();i++) {
                x0[i] = Optimizable::getParameter(i).value();
            }
            //Optimization init
            libcmaes::CMAParameters<> cmaparams(x0, 0.1);
            cmaparams.set_quiet(isQuiet);
            cmaparams.set_mt_feval(false);
            cmaparams.set_str_algo("acmaes");
            cmaparams.set_max_iter(maxIteration);
            
            //Fitness function
            libcmaes::FitFunc fitness = 
                [this, beginTimeLearn, endTimeLearn, 
                    beginTimeTest, endTimeTest](const double* x, const int N) 
                {
                    (void)N;
                    //Set parameters
                    this->resetParameters();
                    for (size_t i=0;i<this->parameterSize();i++) {
                        this->setParameter(i, x[i]);
                    }

                    //Reset LWPR model
                    resetRegression();
                    //Train LWMR
                    if (this->rangeLearn(
                        beginTimeLearn, endTimeLearn) == false
                    ) {
                        throw std::runtime_error(
                            "Regression no learning point");
                    }
                    //Test LWPR
                    double mse = this->rangeMSE(beginTimeTest, endTimeTest);
                    if (mse < 0.0) {
                        throw std::runtime_error(
                            "Regression no testing point");
                    }

                    //Penalize unbounded parameters
                    double cost = mse;
                    for (size_t i=0;i<this->parameterSize();i++) {
                        if (
                            this->getParameter(i).hasMinimum() &&
                            x[i] < this->getParameter(i).getMinimum()
                        ) {
                            cost += 100.0 + 1000.0*fabs(
                                x[i] - this->getParameter(i).getMinimum());
                        }
                        if (
                            this->getParameter(i).hasMaximum() &&
                            x[i] > this->getParameter(i).getMaximum()
                        ) {
                            cost += 100.0 + 1000.0*fabs(
                                x[i] - this->getParameter(i).getMaximum());
                        }
                    }

                    return cost;
                };

            //Run optimization
            libcmaes::CMASolutions cmasols = 
                libcmaes::cmaes<>(fitness, cmaparams);
            Eigen::VectorXd bestParams = 
                cmasols.best_candidate().get_x_dvec();

            //Set founded parameters
            for (size_t i=0;i<parameterSize();i++) {
                Optimizable::setParameter(i, bestParams(i));
            }
        }

    protected:
        
        /**
         * Handler called when a new input
         * is added to the regression
         */
        virtual void onAddInput() = 0;

    private:

        /**
         * Inputs series container and output series.
         * The regression try to learn the output at time t
         * with given input retrieved at time t - deltaTime.
         */
        std::vector<Input> _inputSeries;
        const TimeSeries* _outputSeries;
};

}

#endif

