#include <iostream> //TODO
#include <stdexcept>
#include "Gradient/FiniteDifferenceGradient.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"

namespace Leph {

void FiniteDifferenceGradient::addExperiment(
    const Vector& deltaParam, double fitness)
{
    if (_deltaParams.size() > 0 && (size_t)deltaParam.size() != dimension()) {
        throw std::logic_error(
            "FiniteDifferenceGradient invalid dimension");
    }

    _fitnesses.push_back(fitness);
    _deltaParams.push_back(deltaParam);

    update();
}
        
size_t FiniteDifferenceGradient::size() const
{
    return _fitnesses.size();
}
        
size_t FiniteDifferenceGradient::dimension() const
{
    if (_deltaParams.size() > 0) {
        return _deltaParams.front().size();
    } else {
        return 0;
    }
}
        
Vector FiniteDifferenceGradient::gradient() const
{
    return _gradient;
}

void FiniteDifferenceGradient::update()
{
    static Vector tmp(3); //TODO;
    SimpleLinearRegression regression;

    for (size_t i=0;i<size();i++) {
        double weight = 1.0;
        Vector input = _deltaParams[i];
        input.conservativeResize(dimension()+1, Eigen::NoChange_t());
        input(dimension()) = 1.0;
        /*
        if (size() >= 5) {
            weight = fabs(_fitnesses[i] - tmp.dot(input));
            weight = exp(-weight/20.0);
            std::cout << "weight: " << weight << std::endl;
        }
        */
        regression.add(input, _fitnesses[i], weight);
    }

    Vector gradient = regression.regression();
    tmp = gradient;
    std::cout << "SIZE: " << size() << std::endl;
    std::cout << "parameters: " << std::endl << gradient << std::endl;
    std::cout << "meanSquaredError:" << regression.meanSquaredError() << std::endl;
    std::cout << "rootMeanSquaredError:" << regression.rootMeanSquaredError() << std::endl;
    std::cout << "correlationCoefficient: " << regression.correlationCoefficient() << std::endl;
    std::cout << "variance:" << regression.variance() << std::endl;
    std::cout << "deviation:" << sqrt(regression.variance()) << std::endl;

    Vector tmpGradient = regression.parameters().head(dimension());
    if (size() > 1) {
        std::cout << "DIFF=" << (_gradient-tmpGradient).norm() << std::endl;
        Vector ref(2);
        ref << 10, 8;
        std::cerr << size() << " " << (_gradient-tmpGradient).norm() << " " << (_gradient-ref).norm() << " " << regression.rootMeanSquaredError() << " " << regression.correlationCoefficient() << " " << _gradient(0) << " " << _gradient(1) << std::endl;
    }
    _gradient = tmpGradient;
}

}

