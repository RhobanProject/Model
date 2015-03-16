#include <iostream>
#include "Spline/SmoothSpline.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::SmoothSpline spline;
    spline.addPoint(0.0, 0.0, 0.0, 0.0);
    spline.addPoint(1.0, 1.0, 0.0, 0.0);
    spline.addPoint(2.0, 0.5, 0.0, 0.0);

    Leph::Plot plot;
    for (double t=-0.5;t<=2.5;t+=0.01) {
        plot.add(Leph::VectorLabel(
            "t", t,
            "pos", spline.pos(t),
            "vel", spline.vel(t),
            "acc", spline.acc(t)
        ));
    }
    plot.plot("t", "all").render();

    return 0;
}

