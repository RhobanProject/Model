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
    dynamicParams("step") = 10;
        
    std::cout << outputs << std::endl;
    std::cout << staticParams << std::endl;
    std::cout << dynamicParams << std::endl;

    Leph::Plot plot;
    for (size_t i=0;i<50*5;i++) {
        walk.exec(0.02, dynamicParams, staticParams);
        plot.add(walk.lastOutputs() & walk.lastInfo());
    }
    plot.plot("index", "all").render();

    std::cout << "CartWalk static params typical deltas" << std::endl;
    std::cout << walk.buildStaticParamsDelta() << std::endl;

    return 0;
}

