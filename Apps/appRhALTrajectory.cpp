#include <iostream>
#include <thread>
#include <chrono>
#include <RhAL.hpp>

int main()
{
    //Initialize the Manager
    RhAL::StandardManager manager;
    manager.readConfig("django_rhal.json");
    std::cout << "Scanning the bus..." << std::endl;
    manager.scan();
    
    //Start the Manager
    std::cout << "Starting Manager Thread" << std::endl;
    manager.startManagerThread();
    std::cout << "Starting RhIO binding" << std::endl;
    RhAL::RhIOBinding binding(manager);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

