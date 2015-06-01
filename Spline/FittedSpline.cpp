#include <algorithm>
#include "Spline/FittedSpline.hpp"
#include "Spline/PolyFit.hpp"
#include "LinearRegression/SimpleLinearRegression.hpp"
#include "Utils/NewtonBinomial.hpp"

namespace Leph {

void FittedSpline::addPoint(double x, double y)
{
    _points.push_back({x, y});
}
        
bool FittedSpline::fittingPieces(double maxError, bool throwError)
{
    //Sort data
    prepareData();

    //Cut x axis into sequences
    //(select spline knots points)
    //by founding data extremum
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
            PolyFit fit(degree);
            for (size_t j=parts[i].first;j<=parts[i].second;j++) {
                fit.add(
                    _points[j].first - _points[parts[i].first].first, 
                    _points[j].second);
            }
            Polynom polynom = fit.fitting();
            //Compute max fitting error
            double error = fit.regression().maxError();
            //Store best fit
            if (bestDegree == 0 || bestError > error) {
                bestDegree = degree;
                bestError = error;
                bestParams = fit.regression().parameters();
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
 
void FittedSpline::fittingGlobal(
    unsigned int degree, unsigned int sequenceLength)
{
    //Sort data
    prepareData();

    //Choose spline knots uniformally
    std::vector<double> knots;
    for (size_t i=1;i<_points.size()-sequenceLength/2;i++) {
        if (i%sequenceLength == 0) {
            knots.push_back(_points[i].first);
        }
    }

    //Prepare smooth spline linear regression with position
    //and derivatives continuity
    //Data are: 1, x, x^2, ..., x^d, (x-knot1)^d, (x-knot2)^d, ...
    SimpleLinearRegression regression;
    for (size_t i=0;i<_points.size();i++) {
        Eigen::VectorXd inputs(degree + 1 + knots.size());
        double expT = 1.0;
        for (size_t k=0;k<degree+1;k++) {
            inputs(k) = expT;
            expT *= _points[i].first - _points.front().first;
        }
        for (size_t k=0;k<knots.size();k++) {
            if (_points[i].first < knots[k]) {
                inputs(degree+1+k) = 0.0;
            } else {
                inputs(degree+1+k) = 
                    pow(_points[i].first - knots[k], degree);
            }
        }
        regression.add(inputs, _points[i].second);
    }

    //Run regression
    regression.regression();

    //Add first spline part
    Polynom polynomFirst(degree);
    for (size_t i=0;i<degree+1;i++) {
        polynomFirst(i) += regression.parameters()(i);
    }
    //Add the spline to the container
    double boundMinFirst = _points.front().first;
    double boundMaxFirst;
    if (knots.size() > 0) {
        boundMaxFirst = knots.front();
    } else {
        boundMaxFirst = _points.back().first;
    }
    Spline::_splines.push_back({
        polynomFirst, boundMinFirst, boundMaxFirst});
    //Add all remaining spline parts beginning by on a knot
    for (size_t k=0;k<knots.size();k++) {
        Polynom polynom(degree);
        for (size_t i=0;i<degree+1;i++) {
            polynom(i) += regression.parameters()(i);
        }
        //Convertion from (x-knot)^d form to
        //coefficient values
        for (size_t i=0;i<=k;i++) {
            Polynom tmp = NewtonBinomial::
                expandPolynom(_points.front().first-knots[i], degree);
            tmp *= regression.parameters()(degree+1+i);
            polynom += tmp;
        }
        //Add it to Spline container 
        double boundMin = knots[k];
        double boundMax;
        if (k == knots.size()-1) {
            boundMax = _points.back().first;
        } else {
            boundMax = knots[k+1];
        }
        //Shift polynom on x axis for spline interface compliance
        polynom.shift(knots[k]-_points.front().first);
        Spline::_splines.push_back({
            polynom, boundMin, boundMax});
    }
}
        
void FittedSpline::prepareData()
{
    //Data check
    if (_points.size() < 3) {
        throw std::logic_error(
            "FittedSpline not enough points");
    }
    
    //Clear spline
    Spline::_splines.clear();

    //Sort all points on x axis
    std::sort(
        _points.begin(), 
        _points.end(), 
        [](const Point& p1, const Point& p2) -> bool { 
            return p1.first < p2.first;
        });
}

}

