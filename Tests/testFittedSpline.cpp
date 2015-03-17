#include <iostream>
#include <fstream>
#include <cmath>
#include "Spline/FittedSpline.hpp"
#include "Plot/Plot.hpp"

double function(double t)
{
    return sin(5.0*t*sin(8.0*t*t))-exp(-t);
}

int main() 
{
    Leph::FittedSpline spline;

    Leph::Plot plot;
    for (double t=0;t<1.0;t+=0.01) {
        spline.addPoint(t, function(t));
        plot.add(Leph::VectorLabel(
            "t", t, 
            "target", function(t)
        ));
    }
    
    spline.fitting(0.01, false);
    for (double t=0;t<1.0;t+=0.01) {
        plot.add(Leph::VectorLabel(
            "t", t, 
            "fitted", spline.pos(t)
        ));
    }

    plot.plot("t", "all").render();

    std::ofstream fileOut("/tmp/testSpline.csv");
    spline.exportData(fileOut);
    fileOut.close();
    
    Leph::Spline splineImport;
    std::ifstream fileIn("/tmp/testSpline.csv");
    splineImport.importData(fileIn);
    fileIn.close();
    splineImport.exportData(std::cout);

    return 0;
}

