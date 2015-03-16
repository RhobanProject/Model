#ifndef LEPH_POLYNOM_HPP
#define LEPH_POLYNOM_HPP

#include <cstdlib>
#include <vector>

namespace Leph {

/**
 * Polynom
 *
 * Simple one dimentional 
 * polynom class for spline 
 * generation
 */
class Polynom
{
    public:

        /**
         * Access to coefficient
         * indexed from constant to
         * higher degree
         */
        const std::vector<double>& getCoefs() const;
        std::vector<double>& getCoefs();

        /**
         * Return polynom degree
         * -1 mean empty polynom
         */
        size_t degree() const;

        /**
         * Polynom evaluation, its first and
         * second derivative at given x
         */
        double pos(double x) const;
        double vel(double x) const;
        double acc(double x) const;

    private:

        /**
         * Polynom coeficients
         */
        std::vector<double> _coefs;
};

}

#endif

