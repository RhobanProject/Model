#ifndef LEPH_SMOOTHSPLINE_HPP
#define LEPH_SMOOTHSPLINE_HPP

#include <vector>
#include <stdexcept>
#include <algorithm>

namespace Leph {

/**
 * SmoothSpline
 *
 * Implementation of 5th order polynomial
 * splines trajectory known to minimize jerk
 */
class SmoothSpline 
{
    public:

        /**
         * Add a new point with its time, position value,
         * velocity and acceleration
         */
        inline void addPoint(double time, double position, 
            double velocity = 0.0, double acceleration = 0.0)
        {
            _points.push_back({time, position, 
                velocity, acceleration});
            computeSplines();
        }
        
        /**
         * Empty all added points and splines
         */
        inline void clear()
        {
            _points.clear();
            _splines.clear();
        }
        
        /**
         * Return the spline interpolation 
         * for given x position, velocity and acceleration
         */
        inline double pos(double x) const
        {
            return interpolation(x, polynomValue);
        }
        inline double vel(double x) const
        {
            return interpolation(x, polynomFirstDiff);
        }
        inline double acc(double x) const
        {
            return interpolation(x, polynomSecondDiff);
        }
        
        /**
         * Return the spline interpolation value
         * with x bound between 0 and 1 of position,
         * velocity and acceleration
         */
        double posMod(double x) const
        {
            return interpolationMod(x, polynomValue);
        }
        double velMod(double x) const
        {
            return interpolationMod(x, polynomFirstDiff);
        }
        double accMod(double x) const
        {
            return interpolationMod(x, polynomSecondDiff);
        }
        

    private:

        /**
         * Simple point struture
         */
        struct Point {
            double time;
            double position;
            double velocity;
            double acceleration;
        };

        /**
         * Internal 5th order polynom degree
         */
        struct Polynom {
            double coef[6];
        };

        /**
         * Internal spline representation
         */
        struct Spline {
            Polynom p;
            double min;
            double max;
        };

        /**
         * Points container
         */
        std::vector<Point> _points;

        /**
         * Splines container
         */
        std::vector<Spline> _splines;

        /**
         * Evaluate the given polynom at given
         * time
         */
        static inline double polynomValue(
            const Polynom& p, double x)
        {
            double val = 0.0;
            double xx = 1.0;
            for (int i=0;i<=5;i++) {
                val += p.coef[i]*xx;
                xx *= x;
            }
            return val;
        }

        /**
         * Evaluate the first derivative of given
         * polynom at given time
         */
        static inline double polynomFirstDiff(
            const Polynom& p, double x)
        {
            double val = 0.0;
            double xx = 1.0;
            for (int i=1;i<=5;i++) {
                val += (double)(i)*p.coef[i]*xx;
                xx *= x;
            }
            return val;
        }
        
        /**
         * Evaluate the second derivative of given
         * polynom at given time
         */
        static inline double polynomSecondDiff(
            const Polynom& p, double x)
        {
            double val = 0.0;
            double xx = 1.0;
            for (int i=2;i<=5;i++) {
                val += (double)(i*(i-1))*p.coef[i]*xx;
                xx *= x;
            }
            return val;
        }

        /**
         * Fit a polynom between 0 and t with given
         * pos, vel and acc initial and final conditions
         */
        inline Polynom polynomFit(double t, 
            double pos1, double vel1, double acc1,
            double pos2, double vel2, double acc2) const
        {
            if (t <= 0.00001) {
                throw std::logic_error(
                    "SmoothSpline invalid spline interval");
            }
            double t2 = t*t;
            double t3 = t2*t;
            double t4 = t3*t;
            double t5 = t4*t;
            Polynom p;
            p.coef[0] = pos1;
            p.coef[1] = vel1;
            p.coef[2] = acc1/2;
            p.coef[3] = -(-acc2*t2+3*acc1*t2+8*vel2*t+12*vel1*t-20*pos2+20*pos1)/(2*t3);
            p.coef[4] = (-2*acc2*t2+3*acc1*t2+14*vel2*t+16*vel1*t-30*pos2+30*pos1)/(2*t4);
            p.coef[5] = -(-acc2*t2+acc1*t2+6*vel2*t+6*vel1*t-12*pos2+12*pos1)/(2*t5);

            return p;
        }
        
        /**
         * Recompute splines interpolation model
         */
        inline void computeSplines() 
        {
            _splines.clear();
            if (_points.size() < 2) {
                return;
            }

            std::sort(
                _points.begin(), 
                _points.end(), 
                [](const Point& p1, const Point& p2) -> bool { 
                    return p1.time < p2.time;
                });

            for (size_t i=1;i<_points.size();i++) {
                double time = _points[i].time - _points[i-1].time;
                if (time < 0.00001) {
                    throw std::logic_error("SmoothSpline invalid spline range");
                }
                struct Spline spline = {
                    polynomFit(time,
                        _points[i-1].position, _points[i-1].velocity, _points[i-1].acceleration,
                        _points[i].position, _points[i].velocity, _points[i].acceleration),
                    _points[i-1].time,
                    _points[i].time
                };
                
                _splines.push_back(spline);
            }
        }

        /**
         * Return spline interpolation of given value and
         * used given polynom evaluation function
         */
        double interpolation(double x, 
            double(*func)(const Polynom&, double)) const
        {
            if (_points.size() == 0) {
                return 0.0;
            } else if (_points.size() == 1) {
                return _points.front().position;
            } else {
                for (size_t i=0;i<_splines.size();i++) {
                    if (x >= _splines[i].min && x <= _splines[i].max) {
                        return func(
                            _splines[i].p, x-_splines[i].min);
                    }
                }
                return 0.0;
            }
        }

        /**
         * Return interpolation with x 
         * bound between 0 and 1
         */
        double interpolationMod(double x, 
            double(*func)(const Polynom&, double)) const
        {
            if (x < 0.0) {
                x = 1.0 + (x - ((int)x/1));
            } else if (x > 1.0) {
                x = (x - ((int)x/1));
            }

            return interpolation(x, func);
        }
};

}

#endif

