#include <iostream>
#include <string>
#include <chrono>
#include <thread>

//CODE
#include "Types/VectorLabel.hpp"
#include "CartWalk/CartWalkProxy.hpp"

//UTILS
#include "SDKConnection.hpp"
#include "SDKInterface.hpp"

int main()
{
    //Initialisation of CartWalk parameters
    Leph::CartWalkProxy walk;
    Leph::VectorLabel params = walk.buildParams();

    //Initialize the connection
    Leph::SDKConnection sdkConnection;

    //Initialize the interface
    std::string statusEnabled = "Walk is Disabled";
    Leph::SDKInterface interface(sdkConnection, "CartWalk Tunner");
    interface.addParameters("Dynamic Parameters", params, "dynamic");
    interface.addParameters("Static Parameter", params, "static");
    interface.addStatus(statusEnabled);
    interface.addBinding(' ', "Toggle walk enable", [&params, &statusEnabled](){
        params("dynamic:enabled") = !params("dynamic:enabled");
        if (params("dynamic:enabled")) {
            statusEnabled = "Walk is Enabled";
        } else {
            statusEnabled = "Walk is Disabled";
        }
    });

    //Main loop
    while (interface.tick()) {
        const double freq = 50.0;
        //Generate walk orders
        walk.exec(1.0/freq, params);
        Leph::VectorLabel outputs = walk.lastOutputs();
        //Sending them to the robot
        sdkConnection.setMotorAngles(outputs);
        //Waiting
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(1000/freq)));
    }
    interface.quit();
    
    //Printing last used parameters
    std::cout << params << std::endl;

    return 0;
}

