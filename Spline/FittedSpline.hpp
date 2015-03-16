#ifndef LEPH_FITTEDSPLINE_HPP
#define LEPH_FITTEDSPLINE_HPP

#include <vector>
#include "Spline/Spline.hpp"

namespace Leph {

/**
 * FittedSpline
 *
 * Fit polynomial spline from
 * unnoised data point
 * Each spline part could be fitted by
 * a different degree polynom
 */
class FittedSpline : public Spline
{
    public:

        /**
         * Add a point for fitting
         */
        void addPoint(double x, double y);

        /**
         * Compute fitting spline with registered
         * data points
         * maxError is maximum error allowed between given
         * data point and fitted model prediction
         * if throwError is true, runtime_error is thrown if 
         * no fit is found.
         * Return true if fit is successful
         */
        bool fitting(double maxError, bool throwError);

    private:

        /**
         * Point typedef
         */
        typedef std::pair<double, double> Point;

        /**
         * Fitted point container
         */
        std::vector<Point> _points;
};

}

#endif

