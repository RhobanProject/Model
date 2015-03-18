#include <algorithm>
#include "Spline/FittedSpline.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"

namespace Leph {

void FittedSpline::addPoint(double x, double y)
{
    _points.push_back({x, y});
}
        
bool FittedSpline::fitting(double maxError, bool throwError)
{
    if (_points.size() < 3) {
        throw std::logic_error(
            "FittedSpline not enough points");
    }

    //Sort all points on x axis
    std::sort(
        _points.begin(), 
        _points.end(), 
        [](const Point& p1, const Point& p2) -> bool { 
            return p1.first < p2.first;
        });

    //Cut x axis into sequences
    //(select spline knots points)
    std::vector<std::pair<size_t, size_t>> parts;
    size_t beginIndex = 0;
    bool isIncreasing;
    //Find initial trend
    if (_points[1].second > _points[0].second + 1e-6) {
        isIncreasing = true;
    } else {
        isIncreasing = false;
    }
    for (size_t i=1;i<_points.size();i++) {
        if (
            isIncreasing && 
            _points[i].second < _points[i-1].second - 1e-6
        ) {
            parts.push_back({beginIndex, i-1});
            beginIndex = i-1;
            isIncreasing = false;
        } else if (
            !isIncreasing && 
            _points[i].second > _points[i-1].second + 1e-6
        ) {
            parts.push_back({beginIndex, i-1});
            beginIndex = i-1;
            isIncreasing = true;
        }
    }
    parts.push_back({beginIndex, _points.size()-1});

    //Return value
    bool isFitSuccessful = true;

    //Compute linear regression for each parts
    //to find best polynomial fit
    for (size_t i=0;i<parts.size();i++) {
        //Try to fit with polynom of increasing degree
        unsigned int degree = 1;
        //Best fit found for unsuccessful fit
        double bestError = 0;
        unsigned int bestDegree = 0;
        Vector bestParams;
        while (true) {
            //Polynomial simple linear regression
            SimpleLinearRegression regression;
            for (size_t j=parts[i].first;j<=parts[i].second;j++) {
                Vector inputs(degree+1);
                double expVal = 1.0;
                for (size_t k=0;k<degree+1;k++) {
                    inputs(k) = expVal ;
                    expVal *= _points[j].first- _points[parts[i].first].first;
                }
                regression.add(inputs, _points[j].second);
            }
            regression.regression();
            //Compute max fitting error
            double error = regression.maxError();
            //Store best fit
            if (bestDegree == 0 || bestError > error) {
                bestDegree = degree;
                bestError = error;
                bestParams = regression.parameters();
            }
            //Iterations are stopped if error threshold is meet
            //or degree is too high for data points available
            if (
                degree >= parts[i].second-parts[i].first || 
                bestError <= maxError
            ) {
                //If the fit is non successful, we can throw
                //exception or return false value
                if (bestError > maxError) {
                    isFitSuccessful = false;
                    if (throwError) {
                        throw std::runtime_error(
                            "FittedSpline unable to fit data point");
                    }
                }
                //Save computed fitting polynom to Splines container
                Polynom polynom;
                polynom.getCoefs().resize(bestDegree+1);
                for (size_t k=0;k<bestDegree+1;k++) {
                    polynom.getCoefs()[k] = bestParams(k);
                }
                Spline::_splines.push_back({
                    polynom, 
                    _points[parts[i].first].first,
                    _points[parts[i].second].first});
                //Go to next spline part
                break;
            } else {
                //Else, continue iteration with higher degree
                degree++;
            }
        }
    }

    return isFitSuccessful;
}

}

