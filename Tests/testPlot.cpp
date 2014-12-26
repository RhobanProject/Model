#include <iostream>
#include "Types/VectorLabel.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::Plot plot;
    for (int i=0;i<100;i++) {
        Leph::VectorLabel vect({
            std::make_pair("toto", i),
            std::make_pair("titi", -i),
            std::make_pair("tata", 1.0),
            std::make_pair("tete", -0.5*i)
        });
        plot.add(vect);
    }
    plot.add(Leph::VectorLabel("tutu", 0.0));
    plot.add(Leph::VectorLabel("tutu", 0.0));
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "titi", Leph::Plot::Points)
        .plot("index", "all")
        .render();

    return 0;
}

