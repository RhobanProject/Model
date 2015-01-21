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
    Leph::VectorLabel staticParams = walk.buildStaticParams();
    Leph::VectorLabel dynamicParams = walk.buildDynamicParams();

    //Initialize the connection
    Leph::SDKConnection sdkConnection;

    //Initialize the interface
    std::string statusEnabled = "Walk is Disabled";
    Leph::SDKInterface interface(sdkConnection, "CartWalk Tunner");
    interface.addParameters("Dynamic Parameters", dynamicParams);
    interface.addParameters("Static Parameter", staticParams);
    interface.addStatus(statusEnabled);
    interface.addBinding(' ', "Toggle walk enable", [&dynamicParams, &statusEnabled](){
        dynamicParams("enabled") = !dynamicParams("enabled");
        if (dynamicParams("enabled")) {
            statusEnabled = "Walk is Enabled";
        } else {
            statusEnabled = "Walk is Disabled";
        }
    });

    //Main loop
    while (interface.tick()) {
        const double freq = 50.0;
        //Generate walk orders
        walk.exec(1.0/freq, dynamicParams, staticParams);
        Leph::VectorLabel outputs = walk.lastOutputs();
        //Sending them to the robot
        sdkConnection.setMotorAngles(outputs);
        //Waiting
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(1000/freq)));
    }
    interface.quit();
    
    //Printing last used parameters
    std::cout << dynamicParams << std::endl;
    std::cout << staticParams << std::endl;

    return 0;
}

