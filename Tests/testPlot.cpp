#include <iostream>
#include "Types/VectorLabel.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::Plot plot;
    for (int i=0;i<100;i++) {
        Leph::VectorLabel vect(
            "aa:toto", i,
            "aa:titi", -i,
            "bb:tata", 1.0,
            "bb:tete", -0.5*i
        );
        plot.add(vect);
    }
    plot.add(Leph::VectorLabel("cc:tutu", 0.0));
    plot.add(Leph::VectorLabel("cc:tutu", 0.0));
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "aa:*", Leph::Plot::Lines)
        .plot("index", "aa:titi", Leph::Plot::Points)
        .plot("index", "bb:tata", Leph::Plot::Points, "aa:toto")
        .render();
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "aa:titi", Leph::Plot::Points)
        .plot("index", "bb:tata", Leph::Plot::Points, "aa:toto")
        .render("/tmp/testPlot.plot");

    return 0;
}

