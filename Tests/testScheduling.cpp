#include <iostream>
#include "Utils/Scheduling.hpp"
#include "Utils/Chrono.hpp"

int main()
{
    Leph::Scheduling scheduling;
    scheduling.setFrequency(50.0);

    Leph::Chrono chrono;

    chrono.start("loop");
    for (size_t i=0;i<50;i++) {
        chrono.start("iter");
        scheduling.wait();
        chrono.stop("iter");
    }
    chrono.stop("loop");
    chrono.print();
    std::cout << "Duration: " << scheduling.duration() << std::endl;
    std::cout << "IsError: " << scheduling.isError() << " " 
        << scheduling.timeError() << std::endl;

    chrono.clear();
    chrono.start("iter");
    for (volatile size_t i=0;i<100000000;i++) {}
    scheduling.wait();
    chrono.stop("iter");
    chrono.print();
    std::cout << "Duration: " << scheduling.duration() << std::endl;
    std::cout << "IsError: " << scheduling.isError() << " " 
        << scheduling.timeError() << std::endl;

    return 0;
}

