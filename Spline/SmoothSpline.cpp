#include <stdexcept>
#include <algorithm>
#include "Spline/SmoothSpline.hpp"

namespace Leph {

void SmoothSpline::addPoint(double time, double position, 
    double velocity, double acceleration)
{
    _points.push_back({time, position, 
        velocity, acceleration});
    computeSplines();
}

Polynom SmoothSpline::polynomFit(double t, 
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
    p.getCoefs().resize(6);
    p.getCoefs()[0] = pos1;
    p.getCoefs()[1] = vel1;
    p.getCoefs()[2] = acc1/2;
    p.getCoefs()[3] = -(-acc2*t2+3*acc1*t2+8*vel2*t+12*vel1*t-20*pos2+20*pos1)/(2*t3);
    p.getCoefs()[4] = (-2*acc2*t2+3*acc1*t2+14*vel2*t+16*vel1*t-30*pos2+30*pos1)/(2*t4);
    p.getCoefs()[5] = -(-acc2*t2+acc1*t2+6*vel2*t+6*vel1*t-12*pos2+12*pos1)/(2*t5);

    return p;
}

void SmoothSpline::computeSplines() 
{
    Spline::_splines.clear();
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
        Spline::_splines.push_back({
            polynomFit(time,
                _points[i-1].position, _points[i-1].velocity, _points[i-1].acceleration,
                _points[i].position, _points[i].velocity, _points[i].acceleration),
            _points[i-1].time,
            _points[i].time,
            true
        });
    }
}

}

