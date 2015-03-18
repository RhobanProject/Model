#include <iomanip>
#include <stdexcept>
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
        
double Spline::min() const
{
    if (_splines.size() == 0) {
        return 0.0;
    } else {
        return _splines.front().min;
    }
}
double Spline::max() const
{
    if (_splines.size() == 0) {
        return 0.0;
    } else {
        return _splines.back().max;
    }
}

void Spline::exportData(std::ostream& os) const
{
    for (size_t i=0;i<_splines.size();i++) {
        os << std::setprecision(10) << _splines[i].min << " ";
        os << std::setprecision(10) << _splines[i].max << " ";
        os << std::setprecision(10) << 
            _splines[i].polynom.getCoefs().size() << " ";
        for (size_t j=0;j<_splines[i].polynom.getCoefs().size();j++) {
            os << std::setprecision(10) << 
                _splines[i].polynom.getCoefs()[j] << " ";
        }
    }
    os << std::endl;
}
void Spline::importData(std::istream& is)
{
    bool isFormatError;
    while (is.good()) {
        isFormatError = true;
        double min;
        double max;
        size_t size;
        Polynom p;
        //Load spline interval and degree
        is >> min;
        if (!is.good()) break;
        is >> max;
        if (!is.good()) break;
        is >> size;
        //Load polynom coeficients
        p.getCoefs().resize(size);
        for (size_t i=0;i<size;i++) {
            if (!is.good()) break;
            is >> p.getCoefs()[i];
        }
        //Save spline part
        isFormatError = false;
        _splines.push_back({p, min, max});
        //Exit on line break
        while (is.peek() == ' ') {
            if (!is.good()) break;
            is.ignore();
        }
        if (is.peek() == '\n') {
            break;
        }
    }
    if (isFormatError) {
        throw std::logic_error(
            "Spline import format invalid");
    }
}

double Spline::interpolation(double x, 
    double(Polynom::*func)(double) const) const
{
    for (size_t i=0;i<_splines.size();i++) {
        if (x >= _splines[i].min && x <= _splines[i].max) {
            return (_splines[i].polynom.*func)
                (x-_splines[i].min);
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

