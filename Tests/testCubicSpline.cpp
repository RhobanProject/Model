#include <iostream>
#include "Spline/CubicSpline.hpp"
#include "Spline/LinearSpline.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::CubicSpline spline;
    Leph::LinearSpline splineLin;
    spline.addPoint(0.0, 0.0, 0.0);
    spline.addPoint(1.0, 1.0, 0.0);
    spline.addPoint(2.0, 0.5, 0.0);
    spline.addPoint(2.0, 0.5, 4.0);
    spline.addPoint(3.0, -2.0, 0.0);
    splineLin.addPoint(0.0, 0.0);
    splineLin.addPoint(1.0, 1.0);
    splineLin.addPoint(2.0, 0.5);
    splineLin.addPoint(2.0, 0.5);
    splineLin.addPoint(3.0, -2.0);

    Leph::Plot plot;
    for (double t=-0.5;t<=3.5;t+=0.01) {
        plot.add(Leph::VectorLabel(
            "t", t,
            "pos", spline.pos(t),
            "pos linear", splineLin.pos(t),
            "vel", spline.vel(t),
            "acc", spline.acc(t)
        ));
    }
    plot.plot("t", "all").render();

    return 0;
}

