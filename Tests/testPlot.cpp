#include <iostream>
#include "Types/VectorLabel.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::Plot plot;
    for (int i=0;i<100;i++) {
        Leph::VectorLabel vect(
            "toto", i,
            "titi", -i,
            "tata", 1.0,
            "tete", -0.5*i
        );
        plot.add(vect);
    }
    plot.add(Leph::VectorLabel("tutu", 0.0));
    plot.add(Leph::VectorLabel("tutu", 0.0));
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "titi", Leph::Plot::Points)
        .plot("index", "tata", Leph::Plot::Points, "toto")
        .render();
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "titi", Leph::Plot::Points)
        .plot("index", "tata", Leph::Plot::Points, "toto")
        .render("/tmp/plot");

    return 0;
}

