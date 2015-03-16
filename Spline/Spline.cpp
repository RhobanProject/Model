#include "Spline/Spline.hpp"

namespace Leph {

double Spline::pos(double t) const
{
    return interpolation(t, &Polynom::pos);
}
double Spline::vel(double t) const
{
    return interpolation(t, &Polynom::vel);
}
double Spline::acc(double t) const
{
    return interpolation(t, &Polynom::acc);
}
        
double Spline::posMod(double t) const
{
    return interpolationMod(t, &Polynom::pos);
}
double Spline::velMod(double t) const
{
    return interpolationMod(t, &Polynom::vel);
}
double Spline::accMod(double t) const
{
    return interpolationMod(t, &Polynom::acc);
}

double Spline::interpolation(double x, 
    double(Polynom::*func)(double) const) const
{
    for (size_t i=0;i<_splines.size();i++) {
        if (x >= _splines[i].min && x <= _splines[i].max) {
            if (_splines[i].isNormalization) {
                return (_splines[i].polynom.*func)
                    (x-_splines[i].min);
            } else {
                return (_splines[i].polynom.*func)(x);
            }
        }
    }
    return 0.0;
}

double Spline::interpolationMod(double x, 
    double(Polynom::*func)(double) const) const
{
    if (x < 0.0) {
        x = 1.0 + (x - ((int)x/1));
    } else if (x > 1.0) {
        x = (x - ((int)x/1));
    }
    return interpolation(x, func);
}

}

