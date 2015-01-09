#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

//SDK
#include <main/Command.h>
#include <robot/Robot.h>

int main()
{
    try {
        Rhoban::Robot robot(new CommandsStore, "local");
        robot.connect("127.0.0.1", 7777);
        robot.loadEnvironment("/home/rhoban/Environments/RobotBoard/Mowgly/");
        Rhoban::Motors* motors = robot.getMotors();

        std::this_thread::sleep_for(std::chrono::seconds(4));
        motors->get("Coude G")->setRelAngle(0.0);
        std::this_thread::sleep_for(std::chrono::seconds(4));

        for (double t=0.0;t<=5.0;t+=0.02) {
            motors->get("Coude G")->setRelAngle(5.0*sin(2.0*M_PI*t));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::this_thread::sleep_for(std::chrono::seconds(4));
        motors->get("Coude G")->setRelAngle(0.0);
    } catch (std::string err) {
        std::cout << "Exception : " << err << std::endl;
    }

    return 0;
}

