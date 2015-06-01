#include <iostream>
#include <Eigen/Dense>
#include "Plot/Plot.hpp"
#include "gp.h"
#include "rprop.h"

std::default_random_engine generator;      
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

double noise(double gain)
{
    return gain*uniform(generator);
}

double function(double x, double y)
{
    return sin(10*x*x)*y+0.5*y*x+x*x - sin(6*y);
}

int main()
{
    //Plot real target function
    Leph::Plot plot;
    for (double x=0.0;x<=1.0;x+=0.03) {
        for (double y=0.0;y<=1.0;y+=0.03) {
            plot.add(Leph::VectorLabel(
                "x", x, 
                "y", y, 
                "real", function(x, y)));
        }
    }

    //Initialize Gaussian Process
    libgp::GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
    
    //Adding noised data point to model
    for (size_t k=0;k<100;k++) {
        double x = 0.5*uniform(generator) + 0.5;
        double y = 0.5*uniform(generator) + 0.5;
        double f = function(x, y) + noise(1.0);
        plot.add(Leph::VectorLabel(
            "x", x, 
            "y", y, 
            "target", f));
        double input[] = {x, y};
        gp.add_pattern(input, f);
    }
    
    //Optimize hyper parameter
    libgp::RProp rprop;
    rprop.init();
    rprop.maximize(&gp, 50, true);
    
    //Predict fitted model
    for (double x=0.0;x<=1.0;x+=0.04) {
        for (double y=0.0;y<=1.0;y+=0.04) {
            double input[] = {x, y};
            plot.add(Leph::VectorLabel(
                "x", x, 
                "y", y, 
                "fitted", gp.f(input)));
        }
    }
    
    //Display plot
    plot.plot("x", "y", "all").render();

    return 0;
}

