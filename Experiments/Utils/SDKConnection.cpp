#include <stdexcept>
#include <chrono>
#include <thread>
#include "SDKConnection.hpp"

namespace Leph {
        
SDKConnection::SDKConnection() :
    _robot(new CommandsStore, "local")
{
    try {
        _robot.connect("127.0.0.1", 7777);
        if (!_robot.isConnected()) {
            throw std::runtime_error("SDKConnection no connection");
        }
        
        _robot.loadEnvironment("/home/rhoban/Environments/RhobanServer/Mowgly");
        
        Rhoban::Motors* motors = _robot.getMotors();
        motors->start(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (const std::string exception) {
        throw std::runtime_error("SDKConnection exception: " + exception);
    }
}

SDKConnection::~SDKConnection()
{
    Rhoban::Motors* motors = _robot.getMotors();
    motors->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
        
void SDKConnection::setMotorAngles(const Leph::VectorLabel& outputs)
{
    Rhoban::Motors* motors = _robot.getMotors();
    motors->get("Pied G")->setAngle(outputs("left foot roll"));
    motors->get("Cheville G")->setAngle(outputs("left foot pitch"));
    motors->get("Genou G")->setAngle(outputs("left knee"));
    motors->get("Cuisse G")->setAngle(outputs("left hip pitch"));
    motors->get("Hanche G Lat")->setAngle(outputs("left hip roll"));
    motors->get("Hanche G Rot")->setAngle(outputs("left hip yaw"));
    motors->get("Pied D")->setAngle(outputs("right foot roll"));
    motors->get("Cheville D")->setAngle(outputs("right foot pitch"));
    motors->get("Genou D")->setAngle(outputs("right knee"));
    motors->get("Cuisse D")->setAngle(outputs("right hip pitch"));
    motors->get("Hanche D Lat")->setAngle(outputs("right hip roll"));
    motors->get("Hanche D Rot")->setAngle(outputs("right hip yaw"));
}
        
Leph::VectorLabel SDKConnection::getMotorAngles()
{
    Rhoban::Motors* motors = _robot.getMotors();
    return Leph::VectorLabel(
        "left foot roll", motors->get("Pied G")->getRelAngle(),
        "left foot pitch", motors->get("Cheville G")->getRelAngle(),
        "left knee", motors->get("Genou G")->getRelAngle(),
        "left hip pitch", motors->get("Cuisse G")->getRelAngle(),
        "left hip roll", motors->get("Hanche G Lat")->getRelAngle(),
        "left hip yaw", motors->get("Hanche G Rot")->getRelAngle(),
        "right foot roll", motors->get("Pied D")->getRelAngle(),
        "right foot pitch", motors->get("Cheville D")->getRelAngle(),
        "right knee", motors->get("Genou D")->getRelAngle(),
        "right hip pitch", motors->get("Cuisse D")->getRelAngle(),
        "right hip roll", motors->get("Hanche D Lat")->getRelAngle(),
        "right hip yaw", motors->get("Hanche D Rot")->getRelAngle());
}

void SDKConnection::init()
{
    _robot.getMotors()->goToInit(5, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
void SDKConnection::compliant()
{
    _robot.emergency();
}

}

