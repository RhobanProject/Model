#include <iostream>
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::CartWalkProxy walk;
    Leph::VectorLabel outputs = walk.buildOutputs();
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();

    dynamicParams("enabled") = 1;
    
    std::cout << outputs << std::endl;
    std::cout << staticParams << std::endl;
    std::cout << dynamicParams << std::endl;

    Leph::Plot plot;
    for (size_t i=0;i<50*5;i++) {
        outputs = walk.exec(0.02, dynamicParams, staticParams);
        plot.add(outputs);
    }
    plot.plot("index", "all").render();

    return 0;
}

