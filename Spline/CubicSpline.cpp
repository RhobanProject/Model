#include <stdexcept>
#include <algorithm>
#include <random>
#include <chrono>
#include "Spline/CubicSpline.hpp"

namespace Leph {

void CubicSpline::addPoint(double time, double position, 
    double velocity)
{
    _points.push_back({time, 
        position, velocity});
    computeSplines();
}

void CubicSpline::randomNoise(
    double stdDevPos, double stdDevVel, bool updateBounds)
{
    //Initialize the generator
    std::mt19937 generator(static_cast<long unsigned int>(
        std::chrono::high_resolution_clock::now()
        .time_since_epoch().count())); 
    std::normal_distribution<double> distPos(0.0, stdDevPos);
    std::normal_distribution<double> distVel(0.0, stdDevVel);
    //Apply noise on points
    for (size_t i=0;i<_points.size();i++) {
        if (!updateBounds && (i == 0 || i == _points.size()-1)) {
            continue;
        }
        _points[i].position += distPos(generator);
        _points[i].velocity += distVel(generator);
    }
    //Recompute the splines
    computeSplines();
}
        
void CubicSpline::importCallBack()
{
    size_t size = Spline::_splines.size();
    if (size == 0) {
        return;
    }

    double tBegin = Spline::_splines.front().min;
    _points.push_back({
        tBegin, Spline::pos(tBegin), Spline::vel(tBegin)});

    for (size_t i=1;i<size;i++) {
        double t1 = Spline::_splines[i-1].max;
        double t2 = Spline::_splines[i].min;
        double pos1 = Spline::pos(t1);
        double vel1 = Spline::pos(t1);
        double pos2 = Spline::pos(t2);
        double vel2 = Spline::pos(t2);

        if (
            fabs(t2-t1) < 0.0001 && 
            fabs(pos2-pos1) < 0.0001 &&
            fabs(vel2-vel1) < 0.0001
        ) {
            _points.push_back({t1, pos1, vel1});
        } else {
            _points.push_back({t1, pos1, vel1});
            _points.push_back({t2, pos2, vel2});
        }
    }

    double tEnd = Spline::_splines.back().max;
    _points.push_back({
        tEnd, Spline::pos(tEnd), Spline::vel(tEnd)});
}

Polynom CubicSpline::polynomFit(double t, 
    double pos1, double vel1,
    double pos2, double vel2) const
{
    if (t <= 0.00001) {
        throw std::logic_error(
            "CubicSpline invalid spline interval");
    }
    double t2 = t*t;
    double t3 = t2*t;
    Polynom p;
    p.getCoefs().resize(4);
    p.getCoefs()[0] = pos1;
    p.getCoefs()[1] = vel1;
    p.getCoefs()[3] = (vel2 - vel1 - 2.0*(pos2-pos1-vel1*t)/t)/t2;
    p.getCoefs()[2] = (pos2 - pos1 - vel1*t - p.getCoefs()[3]*t3)/t2;

    return p;
}

void CubicSpline::computeSplines() 
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
        if (time > 0.00001) {
            Spline::_splines.push_back({
                polynomFit(time,
                    _points[i-1].position, _points[i-1].velocity,
                    _points[i].position, _points[i].velocity),
                _points[i-1].time,
                _points[i].time
            });
        }
    }
}

}

