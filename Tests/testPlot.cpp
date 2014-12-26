#include <iostream>
#include "Types/VectorLabel.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::Plot plot;
    for (int i=0;i<100;i++) {
        Leph::VectorLabel vect({
            std::make_pair("toto", i*i),
            std::make_pair("titi", -i),
            std::make_pair("tata", 1.0),
            std::make_pair("tete", -0.5*i)
        });
        plot.add(vect);
    }
    plot.rangeX(0.0, 10.0).rangeY(-2.0, 2.0)
        .plot("index", "titi", Leph::Plot::Points)
        .plot("index", "all")
        .render();

    return 0;
}

