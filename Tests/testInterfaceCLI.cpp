#include <iostream>
#include <chrono>
#include <thread>
#include "Ncurses/InterfaceCLI.hpp"

int main()
{
    Leph::InterfaceCLI cli("Test interface");

    Leph::VectorLabel param1(
        "label 1", 1.0,
        "label 2", 2.0,
        "label 3", 3.0);
    Leph::VectorLabel param2(
        "label2 1", 4.0,
        "label2 2", 5.0);
    Leph::VectorLabel values1(
        "value 1", 1.0,
        "value 2", 2.0,
        "value 3", 3.0);
    Leph::VectorLabel values2(
        "value 1", 4.0,
        "value 2", 5.0);

    cli.addParameters("Params 1", param1);
    cli.addParameters("Params 2", param2);
    cli.addMonitors("Values 1", values1);
    cli.addMonitors("Values 2", values2);

    std::string status1 = "Pouet";
    std::string status2 = "Incr Disable";
    cli.addStatus(status1);
    cli.addStatus(status2);
    
    bool doIncr = false;
    cli.addBinding('i', "Toggle values incr", [&doIncr, &status2](){
        doIncr = !doIncr;
        if (doIncr) {
            status2 = "Incr Enabled";
        } else {
            status2 = "Incr Disabled";
        }
    });

    while (cli.tick()) {
        if (doIncr) {
            values1(1) += 0.1;
            values1(2) -= 0.2;
            values2(0) *= 1.01;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(20));
    }
        
    return 0;
}

