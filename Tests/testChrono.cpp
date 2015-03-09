#include <iostream>
#include "Utils/Chrono.hpp"

void wait()
{
    for (volatile long i=0;i<10000000;i++) {}
}

int main()
{
    Leph::Chrono c;
    c.start("test1");
        c.start("test2");
        wait();
        c.stop("test2");
        wait();
    c.stop("test1");
    c.start("test3");
        wait();
        c.start("test4");
        wait();
        c.stop("test4");
        wait();
        c.start("test4");
        wait();
        c.stop("test4");
        wait();
        c.start("test5");
        wait();
        c.stop("test5");
    c.stop("test3");
    c.print();

    return 0;
}

