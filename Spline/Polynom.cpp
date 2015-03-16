#include "Spline/Polynom.hpp"

namespace Leph {

const std::vector<double>& Polynom::getCoefs() const
{
    return _coefs;
}
std::vector<double>& Polynom::getCoefs()
{
    return _coefs;
}

double Polynom::pos(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=0;i<_coefs.size();i++) {
        val += xx*_coefs[i];
        xx *= x;
    }
    return val;
}
double Polynom::vel(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=1;i<_coefs.size();i++) {
        val += i*xx*_coefs[i];
        xx *= x;
    }
    return val;
}
double Polynom::acc(double x) const
{
    double xx = 1.0;
    double val = 0.0;
    for (size_t i=2;i<_coefs.size();i++) {
        val += (i-1)*i*xx*_coefs[i];
        xx *= x;
    }
    return val;
}

}

