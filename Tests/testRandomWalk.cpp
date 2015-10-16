#include <iostream>
#include "Utils/RandomWalk.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::RandomWalk randomWalk(3);

    Leph::Plot plot;
    double sum = 0.0;
    double count = 0.0;
    for (size_t k=0;k<1000;k++) {
        randomWalk.step(1.0, 0.9);
        plot.add(Leph::VectorLabel(
            "t", k, 
            "x", randomWalk.statePos()(0),
            "y", randomWalk.statePos()(1),
            "z", randomWalk.statePos()(2)));
        sum += fabs(randomWalk.stateVel()(0));
        count++;
    }
    std::cout << "Mean delta: " << sum/count << std::endl;
    plot.plot("x", "y", "z", Leph::Plot::LinesPoints, "t").render();

    return 0;
}

