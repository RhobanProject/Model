#include <iostream>
#include "Utils/Chrono.hpp"

int main()
{
    Leph::Chrono c;
    c.start();
    for (volatile long i=0;i<10000000;i++) {}
    c.stop();
    c.print();
    std::cout << std::endl;

    c.clear();
    c.start("time 1");
    for (volatile long i=0;i<10000000;i++) {}
    c.start("time 2");
    for (volatile long i=0;i<10000000;i++) {}
    c.stop("delay 1", "time 1");
    for (volatile long i=0;i<10000000;i++) {}
    c.stop("delay 2", "time 1");
    c.stop("delay 3", "time 2");
    std::cout << "DelaySec: " << c.getDurationSec("delay 1") << std::endl;
    std::cout << "DelayMs : " << c.getDurationMs("delay 1") << std::endl;
    c.print();

    return 0;
}

