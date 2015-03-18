#include <iostream>

//CODE
#include "Spline/SplinesContainer.hpp"
#include "Utils/Scheduling.hpp"

//UTILS
#include "SDKConnection.hpp"
#include "SDKInterface.hpp"

int main(int argc, int argv)
{
    //Command line arguments
    if (argc != 2) {
        std::cout << "Usage: ./app splinesFile" << std::endl;
        return -1;
    }
    std::string splinesFile = argv[1];
    std::cout << "Loading " << splinesFile << std::endl;
    
    //Load splines
    splines.importData(splinesFile);
    
    double freq = 50.0;
    Leph::Scheduling scheduling;
    scheduling.setFrequency(freq);
    while (true) {
        //Waiting
        scheduling.wait();
    }
    
    return 0;
}

