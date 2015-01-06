#include <iostream>
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "CartWalk/CartWalkGradient.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::CartWalkProxy walk;
    Leph::CartWalkGradient gradient;
    
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();
    
    dynamicParams("enabled") = 1;
    dynamicParams("step") = 2.0;

    /*
    Leph::Plot plot;
    double h = 0.1;
    for (int k=0;k<6;k++) {
        Leph::Matrix jacobian = gradient.differentiation(0.9, staticParams, dynamicParams, h);
        Leph::VectorLabel pt;
        pt.append("h", h);
        for (int i=0;i<jacobian.rows();i++) {
            for (int j=0;j<jacobian.cols();j++) {
                pt.append(std::to_string(i)+"_"+std::to_string(j), jacobian(i, j));
            }
        }
        plot.add(pt);
        h /= 10.0;
    }
    plot.plot("index", "all").render();
    */

    Leph::Matrix jacobian = gradient.differentiation(0.9, staticParams, dynamicParams);
    std::cout << walk.buildOutputs() << std::endl;
    std::cout << staticParams << std::endl;
    std::cout << jacobian << std::endl;
    
    return 0;
}

