#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include <lwpr.hh>
#include "Plot/Plot.hpp"
#include <Types/MatrixLabel.hpp>

/**
 * First test of LWPR regression on
 * Sigmaban Logs. Learning prediction and
 * correlation of motors and sensors.
 */

/**
 * Input source
 */
typedef std::vector<std::pair<std::string, size_t>> Inputs;

/**
 * Convertion from and to std::vector to Eigen::Vector
 */
std::vector<double> convert(const Eigen::VectorXd& vect)
{
    std::vector<double> result(vect.size());
    for (size_t i=0;i<(size_t)vect.size();i++) {
        result[i] = vect(i);
    }
    return result;
}
Eigen::VectorXd convert(const std::vector<double>& vect)
{
    Eigen::VectorXd result(vect.size());
    for (size_t i=0;i<(size_t)vect.size();i++) {
        result(i) = vect[i];
    }
    return result;
}
Eigen::VectorXd convert(const double* x, const int n)
{
    Eigen::VectorXd result(n);
    for (size_t i=0;i<(size_t)n;i++) {
        result(i) = x[i];
    }
    return result;
}

/**
 * Load logs data
 */
std::vector<Leph::MatrixLabel> loadLogs()
{
    std::vector<Leph::MatrixLabel> container;

    //Loading data
    Leph::MatrixLabel logs1;
    Leph::MatrixLabel logs2;
    Leph::MatrixLabel logs3;
    Leph::MatrixLabel logs4;
    logs1.load("../../These/Data/logs-2015-05-16/model_2015-05-16-18-51-02.log");
    logs2.load("../../These/Data/logs-2015-05-16/model_2015-05-16-18-51-58.log");
    logs3.load("../../These/Data/logs-2015-05-16/model_2015-05-16-18-49-08.log");
    logs4.load("../../These/Data/logs-2015-05-16/model_2015-05-16-18-49-57.log");
    std::cout << "Loaded Log1 " << logs1.size() 
        << " points with " << logs1.dimension() << " entries" << std::endl;
    std::cout << "Loaded Log2 " << logs2.size() 
        << " points with " << logs2.dimension() << " entries" << std::endl;
    std::cout << "Loaded Log3 " << logs3.size() 
        << " points with " << logs3.dimension() << " entries" << std::endl;
    std::cout << "Loaded Log4 " << logs4.size() 
        << " points with " << logs4.dimension() << " entries" << std::endl;

    container.push_back(logs1);
    container.push_back(logs2);
    container.push_back(logs3);
    container.push_back(logs4);
    return container;
}

/**
 * Return default LWPR meta parameters for given
 * input dimmension
 */
Eigen::VectorXd initLWPRMetaParams(size_t inputDim)
{
    size_t paramDim = 2*inputDim + 5;
    Eigen::VectorXd params(paramDim);

    //NormIn
    for (size_t i=0;i<inputDim;i++) {
        params(i) = 1.0;
    }
    //InitAlpha
    params(inputDim) = 50.0;
    //MetaRate
    params(inputDim + 1) = 250.0;
    //Penalty
    params(inputDim + 2) = 1e-6;
    //InitD
    for (size_t i=0;i<inputDim;i++) {
        params(inputDim + 3 + i) = 100.0;
    }
    //WGen
    params(2*inputDim + 3) = 0.2;
    //WPrune
    params(2*inputDim + 4) = 0.9;

    return params;
}

/**
 * Create and set LWPR meta parameter from
 * given vector
 */
LWPR_Object initLWPR(size_t inputDim, Eigen::VectorXd metaParams)
{
    //InitializeLWPR model with 
    //input and output dimensions
    LWPR_Object model(inputDim, 1);
    
    //Normalisation value for each input dimension (>0)
    std::vector<double> paramsNormIn(inputDim);
    for (size_t i=0;i<inputDim;i++) {
        paramsNormIn[i] = metaParams(i);
    }
    model.normIn(paramsNormIn);
    //Use only diagonal matrix. Big speed up in high dimension
    //but lower performance in complex learning.
    model.diagOnly(true);
    //Learning rate for gradient descente (>0)
    //(meta optimized)
    model.setInitAlpha(metaParams(inputDim));
    //Automatic tunning of distance metric 
    model.useMeta(true);
    //Meta tunning learning rate (>0)
    model.metaRate(metaParams(inputDim + 1));
    //Larger value enforce wider receptive field (>0)
    model.penalty(metaParams(inputDim + 2));
    //Set diagonal (input_dim) or complet (input_dim*input_dim)
    //initial distance matrix (>0)
    std::vector<double> paramsInitD(inputDim);
    for (size_t i=0;i<inputDim;i++) {
        paramsInitD[i] = metaParams(inputDim + 3 + i);
    }
    model.setInitD(paramsInitD);
    //Receptive field activation threshold (>0)
    model.wGen(metaParams(2*inputDim + 3));
    //Receptive field remove threshold
    model.wPrune(metaParams(2*inputDim + 4));

    return model;
}

/**
 * Train given LWPR model with given data logs
 * and using input and output named values
 */
void trainLWPR(LWPR_Object& model, 
    const std::string& output,
    const Inputs& inputs, 
    const std::vector<Leph::MatrixLabel>& container)
{
    size_t inputDim = model.nIn();
    if (inputDim != inputs.size()) {
        throw std::logic_error("Train invalid input");
    }

    std::vector<double> x(inputDim);
    std::vector<double> y(1);
    for (size_t i=0;i<=2;i++) {
        for (size_t j=10;j<container[i].size();j++) {
            for (size_t k=0;k<inputs.size();k++) {
                x[k] = container[i][j - inputs[k].second](inputs[k].first);
            }
            y[0] = container[i][j](output);
            // Update the model with one sample
            model.update(x,y);
        }
    }
}

/**
 * Test LWPR model prediction on unseen data
 * and return mean error
 */
double testLWPR(LWPR_Object& model, 
    const std::string& output,
    const Inputs& inputs, 
    const std::vector<Leph::MatrixLabel>& container)
{
    size_t inputDim = model.nIn();
    if (inputDim != inputs.size()) {
        throw std::logic_error("Test invalid input");
    }

    double mse = 0.0;
    size_t count = 0;
    std::vector<double> x(inputDim);
    std::vector<double> y(1);
    for (size_t j=10;j<container[3].size();j++) {
        for (size_t k=0;k<inputs.size();k++) {
            x[k] = container[3][j - inputs[k].second](inputs[k].first);
        }
        y[0] = container[3][j](output);
        // Predict future
        std::vector<double> yp = model.predict(x);
        mse += pow(yp[0]-y[0], 2);
        count++;
    }

    return sqrt(mse/(double)count);
}

/**
 * Optimize and update given LWPR 
 * meta parametrs using CMAES
 */
void optimizeMetaParameters(
    Eigen::VectorXd& metaParams,
    const std::string& output,
    const Inputs& inputs, 
    const std::vector<Leph::MatrixLabel>& container)
{
    //Starting point
    std::vector<double> x0 = convert(metaParams);
    //Optimization init
    libcmaes::CMAParameters<> cmaparams(x0, 0.1);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("acmaes");
    cmaparams.set_max_iter(10);
    
    //Fitness function
    libcmaes::FitFunc fitness = [&output, &inputs, &container]
        (const double* x, const int N) 
    {
        Eigen::VectorXd params = convert(x, N);
        //Init LWPR model
        LWPR_Object model = initLWPR(inputs.size(), params);
        //Train LWMR
        trainLWPR(model, output, inputs, container);
        //Test LWPR
        double mse = testLWPR(model, output, inputs, container);

        return mse;
    };
    
    //Run optimization
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
    metaParams = cmasols.best_candidate().get_x_dvec();
}

/**
 * Plot test data prediction
 */
void displayTest(LWPR_Object& model, 
    const std::string& output,
    const Inputs& inputs, 
    const std::vector<Leph::MatrixLabel>& container)
{
    size_t inputDim = model.nIn();
    if (inputDim != inputs.size()) {
        throw std::logic_error("Test invalid input");
    }
    
    Leph::Plot plot;
    std::vector<double> x(inputDim);
    std::vector<double> y(1);
    for (size_t j=10;j<container[3].size();j++) {
        for (size_t k=0;k<inputs.size();k++) {
            x[k] = container[3][j - inputs[k].second](inputs[k].first);
        }
        y[0] = container[3][j](output);
        // Predict future
        std::vector<double> yp = model.predict(x);
        plot.add(Leph::VectorLabel(
            "t", container[3][j]("time:timestamp"),
            "y", y[0],
            "yp", yp[0]
        ));
    }
    plot.plot("t", "all").render();
}

int main()
{
    //Input and output definitions
    Inputs inputs = {
        /*
        {"motor:left foot pitch", 4},
        {"motor:left foot roll", 4},
        {"motor:left knee", 4},
        {"motor:left hip yaw", 4},
        {"motor:left hip pitch", 4},
        {"motor:left hip roll", 4},
        {"motor:right foot pitch", 4},
        {"motor:right foot roll", 4},
        {"motor:right knee", 4},
        {"motor:right hip yaw", 4},
        {"motor:right hip pitch", 4},
        {"motor:right hip roll", 4},
        {"sensor:roll", 4},
        {"sensor:pitch", 4},
        */
        /*
        {"motor:left foot pitch", 1},
        {"motor:left foot roll", 1},
        {"motor:left knee", 1},
        {"motor:left hip yaw", 1},
        {"motor:left hip pitch", 1},
        {"motor:left hip roll", 1},
        {"motor:right foot pitch", 1},
        {"motor:right foot roll", 1},
        {"motor:right knee", 1},
        {"motor:right hip yaw", 1},
        {"motor:right hip pitch", 1},
        {"motor:right hip roll", 1},
        {"sensor:roll", 1},
        {"sensor:pitch", 1},
        */
        {"output:left knee", 5},
        {"output:left knee", 6},
        {"output:left knee", 7},
        {"output:left knee", 8}
    };
    std::string output = "motor:left knee";

    //Load data
    std::vector<Leph::MatrixLabel> container = loadLogs();
    //Init LWPR parameters
    Eigen::VectorXd metaParams = initLWPRMetaParams(inputs.size());
    //Optimize parameters with CMAES
    optimizeMetaParameters(metaParams, output, inputs, container);
    //Init LWPR model
    LWPR_Object model = initLWPR(inputs.size(), metaParams);
    //Train LWMR
    trainLWPR(model, output, inputs, container);
    //Test LWPR
    double mse = testLWPR(model, output, inputs, container);
    //Display informations
    std::cout << "MeanError: " << mse << std::endl;
    std::cout << "TrainingData: " << model.nData() << std::endl;
    std::cout << "InputDims: " << model.nIn() << std::endl;
    std::cout << "ReceptiveFields: " << model.numRFS(0) << std::endl;
    std::cout << "MetaParameters: " << metaParams.transpose() << std::endl;
    //Display prediction
    displayTest(model, output, inputs, container);

    return 0;
}

