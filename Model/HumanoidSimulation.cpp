#include <stdexcept>
#include "Model/HumanoidSimulation.hpp"
#include "Utils/Euler.h"

namespace Leph {

HumanoidSimulation::HumanoidSimulation(RobotType type) :
    _model(type, "trunk", true),
    _simulation(_model),
    _constraints(nullptr),
    _cleats()
{
    //Model initialization
    putOnGround();
    putFootAt(0.0, 0.0);
    //Foot cleats initialization
    addCleat("left_cleat_1");
    addCleat("left_cleat_2");
    addCleat("left_cleat_3");
    addCleat("left_cleat_4");
    addCleat("right_cleat_1");
    addCleat("right_cleat_2");
    addCleat("right_cleat_3");
    addCleat("right_cleat_4");
}
        
HumanoidSimulation::~HumanoidSimulation()
{
    if (_constraints != nullptr) {
        delete _constraints;
        _constraints = nullptr;
    }
}

const HumanoidModel& HumanoidSimulation::model() const
{
    return _model;
}
HumanoidModel& HumanoidSimulation::model()
{
    return _model;
}
        
void HumanoidSimulation::putOnGround()
{
    //Find lowest foot
    std::string foot;
    Eigen::Vector3d posLeft = 
        _model.position("trunk", "left_foot_tip");
    Eigen::Vector3d posRight = 
        _model.position("trunk", "right_foot_tip");
    if (posLeft.z() >= posRight.z()) {
        foot = "left_foot_tip";
    } else {
        foot = "right_foot_tip";
    }

    //Set the foot flat
    Eigen::Matrix3d rotOriginToTrunk = 
        _model.orientation("trunk", "origin");
    Eigen::Matrix3d rotOriginToFoot = 
        _model.orientation(foot, "origin");
    Eigen::Matrix3d rotation = 
        rotOriginToTrunk * rotOriginToFoot.transpose();
    //Retrieve YawPitchRoll euler angles from rotation matrix
    //(Manual computing without singular check seems better than
    //Eigen euler angles and with better range)
    Eigen::Vector3d angles;
    //Roll
    angles(0) = atan2(rotation(1, 2), rotation(2, 2));
    //Pitch
    angles(1) = atan2(-rotation(0, 2), 
        sqrt(rotation(0, 0)*rotation(0, 0) 
            + rotation(0, 1)*rotation(0, 1)));
    //Yaw
    angles(2) = atan2(rotation(0, 1), rotation(0, 0));
    //Update base
    setPos("base_roll", angles(0));
    setPos("base_pitch", angles(1));

    //Set the foot on ground
    Eigen::VectorXd pos = _model.position(foot, "origin");
    double height = getPos("base_z");
    setPos("base_z", height-pos.z());
}
        
void HumanoidSimulation::putFootAt(double x, double y)
{
    //Find lowest foot
    std::string foot;
    Eigen::Vector3d posLeft = 
        _model.position("trunk", "left_foot_tip");
    Eigen::Vector3d posRight = 
        _model.position("trunk", "right_foot_tip");
    if (posLeft.z() >= posRight.z()) {
        foot = "left_foot_tip";
    } else {
        foot = "right_foot_tip";
    }
    
    //Update base position
    Eigen::Vector3d posFoot = _model.position(foot, "origin");
    setPos("base_x", getPos("base_x") - posFoot.x());
    setPos("base_y", getPos("base_y") - posFoot.y());
}

const Eigen::VectorXd& HumanoidSimulation::positions() const
{
    return _simulation.positions();
}
Eigen::VectorXd& HumanoidSimulation::positions()
{
    return _simulation.positions();
}
const Eigen::VectorXd& HumanoidSimulation::velocities() const
{
    return _simulation.velocities();
}
Eigen::VectorXd& HumanoidSimulation::velocities()
{
    return _simulation.velocities();
}
const Eigen::VectorXd& HumanoidSimulation::goals() const
{
    return _simulation.goals();
}
Eigen::VectorXd& HumanoidSimulation::goals()
{
    return _simulation.goals();
}
const Eigen::VectorXd& HumanoidSimulation::jointTorques() const
{
    return _simulation.jointTorques();
}
Eigen::VectorXd& HumanoidSimulation::jointTorques()
{
    return _simulation.jointTorques();
}
const Eigen::VectorXd& HumanoidSimulation::accelerations() const
{
    return _simulation.accelerations();
}
Eigen::VectorXd& HumanoidSimulation::accelerations()
{
    return _simulation.accelerations();
}

void HumanoidSimulation::setGoal(
    const std::string& name, double pos)
{
    _simulation.goals()(_model.getDOFIndex(name)) = pos;
}
void HumanoidSimulation::setPos(
    const std::string& name, double pos)
{
    _simulation.positions()(_model.getDOFIndex(name)) = pos;
    _model.setDOF(name, pos);
}
void HumanoidSimulation::setVel(
    const std::string& name, double pos)
{
    _simulation.velocities()(_model.getDOFIndex(name)) = pos;
}

double HumanoidSimulation::getGoal(const std::string& name) const
{
    return _simulation.goals()(_model.getDOFIndex(name));
}
double HumanoidSimulation::getPos(const std::string& name) const
{
    return _simulation.positions()(_model.getDOFIndex(name));
}
double HumanoidSimulation::getVel(const std::string& name) const
{
    return _simulation.velocities()(_model.getDOFIndex(name));
}

const JointModel& HumanoidSimulation::jointModel(
    const std::string& name) const
{
    return _simulation.jointModel(_model.getDOFIndex(name));
}
JointModel& HumanoidSimulation::jointModel(
    const std::string& name)
{
    return _simulation.jointModel(_model.getDOFIndex(name));
}

void HumanoidSimulation::setJointModelParameters(
    const Eigen::VectorXd& params)
{
    _simulation.setJointModelParameters(params);
}

const Eigen::VectorXd& HumanoidSimulation::getCleatForce(
    const std::string& name) const
{
    if (_cleats.count(name) == 0) {
        throw std::logic_error(
            "HumanoidSimulation invalid cleat name: " 
            + name);
    }
    return _cleats.at(name).force;
}

double HumanoidSimulation::getWeightSum() const
{
    double sum = 0.0;
    for (const auto& it : _cleats) {
        sum += it.second.force.z();
    }
    if (sum < 0.0) {
        sum = 0.0;
    }

    return sum/9.81;
}
double HumanoidSimulation::getWeightLeftRatio() const
{
    double sum = 0.0;
    double sumLeft = 0.0;
    for (const auto& it : _cleats) {
        sum += it.second.force.z();
        if (it.second.isLeft) {
            sumLeft += it.second.force.z();
        } 
    }
    if (sumLeft < 0.0) {
        sumLeft = 0.0;
    }
    if (sum < 0.0) {
        sum = 0.0;
    }
   
    if (fabs(sum) < 1e-6) {
        return 0.0;
    } else {
        return sumLeft/sum;
    }
}
double HumanoidSimulation::getWeightRightRatio() const
{
    double sum = 0.0;
    double sumRight = 0.0;
    for (const auto& it : _cleats) {
        sum += it.second.force.z();
        if (!it.second.isLeft) {
            sumRight += it.second.force.z();
        } 
    }
    if (sumRight < 0.0) {
        sumRight = 0.0;
    }
    if (sum < 0.0) {
        sum = 0.0;
    }
   
    if (fabs(sum) < 1e-6) {
        return 0.0;
    } else {
        return sumRight/sum;
    }
}

void HumanoidSimulation::update(double dt)
{
    _simulation.update(dt, _constraints);
    _model.setDOFVect(_simulation.positions());

    bool isAdded = false;
    bool isRemoved = false;
    for (auto& it : _cleats) {
        Eigen::Vector3d pos = _model.position(
            it.second.frame, "origin");
        if (it.second.isEnabled) {
            /* XXX
            if (pos.z() <= 0.0) {
                throw std::logic_error("ERROR CS");
            }
            */
            it.second.force.x() = 
                _constraints->force(3*it.second.index + 0);
            it.second.force.y() = 
                _constraints->force(3*it.second.index + 1);
            it.second.force.z() = 
                _constraints->force(3*it.second.index + 2);
            if (it.second.force.z() < 1e-6) {
                it.second.isEnabled = false;
                isRemoved = true;
            }
        } else {
            it.second.force.setZero();
            if (pos.z() <= 0.0) {
                it.second.isEnabled = true;
                isAdded = true;
                //_simulation.positions()(_model.getDOFIndex("base_z")) += -pos.z() + 1e-8; //XXX
            }
        }
    }
    if (isAdded || isRemoved) {
        buildConstraints();
    }
    if (isAdded) {
        _simulation.velocities() = _model.impulseContacts(
            *_constraints,
            _simulation.positions(),
            _simulation.velocities());
    }
}
        
void HumanoidSimulation::addCleat(const std::string& frame)
{
    _cleats[frame] = {
        frame, (frame.find("left") != std::string::npos), 
        false, 0, Eigen::Vector3d(0.0, 0.0, 0.0)
    };
}
        
void HumanoidSimulation::buildConstraints()
{
    if (_constraints != nullptr) {
        delete _constraints;
        _constraints = nullptr;
    }
    _constraints = new RBDL::ConstraintSet;
    size_t index = 0;
    for (auto& it : _cleats) {
        if (it.second.isEnabled) {
            //Retrieve RBDL body id
            size_t frameId = _model.frameIndexToBodyId(
                _model.getFrameIndex(it.second.frame));
            //Create three constraints for x,y,z axis
            _constraints->AddConstraint(
                frameId,
                Eigen::Vector3d(0.0, 0.0, 0.0),
                RBDLMath::Vector3d(1.0, 0.0, 0.0));
            _constraints->AddConstraint(
                frameId,
                Eigen::Vector3d(0.0, 0.0, 0.0),
                RBDLMath::Vector3d(0.0, 1.0, 0.0));
            _constraints->AddConstraint(
                frameId,
                Eigen::Vector3d(0.0, 0.0, 0.0),
                RBDLMath::Vector3d(0.0, 0.0, 1.0));
            //Assign constraints index
            it.second.index = index;
            index++;
        } 
    }
    _constraints->Bind(_model.getRBDLModel());
}

}

