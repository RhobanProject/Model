#include <iostream>

//SDK
#include <main/Command.h>
#include <rhoban/robot/Robots.h>

int main
{
    try {
        Rhoban::Robots robots;
	robots.loadYaml("config.yml");
        Rhoban::Robot* robot = robots["local"];
        robot->loadMove("/home/rhoban/Environments/RhobanServer/Mowgly/Moves/CartWalk.move.xml", true);
        robot->startMove("CartWalk", 0, 2000);
    } catch (std::string err) {
        std::cout << "Exception : " << err << std::endl;
    }

    return 0;
}

