#include <iostream>
#include <stdexcept>
#include "Model/HumanoidSimulation.hpp"
#include "Utils/Euler.h"
#include "Utils/Combination.hpp"

namespace Leph {

HumanoidSimulation::HumanoidSimulation(
    RobotType type,
    const Eigen::MatrixXd& inertiaData,
    const std::map<std::string, size_t>& inertiaName,
    const Eigen::MatrixXd& geometryData,
    const std::map<std::string, size_t>& geometryName) :
    _model(type, "trunk", true, 
        inertiaData, inertiaName,
        geometryData, geometryName),
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
    //Constraints set initialization
    buildConstraints();
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
    Eigen::Vector3d posLeft = 
        _model.position("trunk", "left_foot_tip");
    Eigen::Vector3d posRight = 
        _model.position("trunk", "right_foot_tip");
    //Forward to putOnGround()
    if (posLeft.z() >= posRight.z()) {
        putOnGround(HumanoidFixedModel::LeftSupportFoot);
    } else {
        putOnGround(HumanoidFixedModel::RightSupportFoot);
    }

}
void HumanoidSimulation::putOnGround(
    HumanoidFixedModel::SupportFoot foot)
{
    std::string footName;
    if (foot == HumanoidFixedModel::LeftSupportFoot) {
        footName = "left_foot_tip";
    } else {
        footName = "right_foot_tip";
    }
    
    //Set the foot flat
    Eigen::Matrix3d rotOriginToTrunk = 
        _model.orientation("trunk", "origin");
    Eigen::Matrix3d rotOriginToFoot = 
        _model.orientation(footName, "origin");
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
    setPos("base_yaw", angles(2));

    //Set the foot on ground
    Eigen::VectorXd pos = _model.position(footName, "origin");
    double height = getPos("base_z");
    setPos("base_z", height-pos.z());
}
        
void HumanoidSimulation::putFootAt(double x, double y)
{
    //Find lowest foot
    Eigen::Vector3d posLeft = 
        _model.position("trunk", "left_foot_tip");
    Eigen::Vector3d posRight = 
        _model.position("trunk", "right_foot_tip");
    //Forward to putFootAt()
    if (posLeft.z() >= posRight.z()) {
        putFootAt(x, y, HumanoidFixedModel::LeftSupportFoot);
    } else {
        putFootAt(x, y, HumanoidFixedModel::RightSupportFoot);
    }

}
void HumanoidSimulation::putFootAt(double x, double y, 
    HumanoidFixedModel::SupportFoot foot)
{
    std::string footName;
    if (foot == HumanoidFixedModel::LeftSupportFoot) {
        footName = "left_foot_tip";
    } else {
        footName = "right_foot_tip";
    }

    //Compute translation in origin frame
    Eigen::Vector3d originToFoot(x, y, 0.0);
    Eigen::Vector3d footToTrunk = _model.position("trunk", footName);
    Eigen::Vector3d originToTrunk = originToFoot + footToTrunk;

    //Update base position
    setPos("base_x", originToTrunk.x());
    setPos("base_y", originToTrunk.y());
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
const Eigen::VectorXd& HumanoidSimulation::frictionTorques() const
{
    return _simulation.frictionTorques();
}
const Eigen::VectorXd& HumanoidSimulation::controlTorques() const
{
    return _simulation.controlTorques();
}
const Eigen::VectorXd& HumanoidSimulation::accelerations() const
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

double HumanoidSimulation::getCleatForce(
    const std::string& name) const
{
    if (_cleats.count(name) == 0) {
        throw std::logic_error(
            "HumanoidSimulation invalid cleat name: " 
            + name);
    }
    return _cleats.at(name).force;
}

void HumanoidSimulation::update(double dt)
{
    //std::cout << "########## Step ######### " << dt << std::endl; //XXX

    //Transition between no contact and contact
    unsigned int countContactLeft = 0;
    unsigned int countContactRight = 0;
    unsigned int countDetachingLeft = 0;
    unsigned int countDetachingRight = 0;
    bool needUpdateLeft = false;
    bool needUpdateRight = false;
    for (auto& it : _cleats) {
        //Compute cleat vertical Z position
        Eigen::Vector3d pos = _model.position(
            it.second.frame, "origin");
        //Check non contacting cleats collision
        if (!it.second.isContact && pos.z() <= -1e-5) {
            //A collision is detected
            it.second.isContact = true;
            it.second.isActive = false;
            //std::cout << "Collision for " << it.second.frame << " Z=" << pos.z() << std::endl; //XXX
            it.second.force = 0.0;
            //Set the foot for constraints update
            if (it.second.isLeft) needUpdateLeft = true;
            else needUpdateRight = true;
        //Check contacting cleats release
        } else if (it.second.isContact && !it.second.isActive && pos.z() > 0.0) {
            //The contact is released
            it.second.isContact = false;
            it.second.isActive = false;
            //std::cout << "Detaching for " << it.second.frame << " Z=" << pos.z() << std::endl; //TODO
            it.second.force = 0.0;
            //Set the foot for constraints update
            if (it.second.isLeft) countDetachingLeft++;
            else countDetachingRight++;
        //Check contacting cleats sinking
        } else if (it.second.isContact && !it.second.isActive && pos.z() < it.second.height - 1e-7) {
            //std::cout << "Sinking for " << it.second.frame << " Z=" << pos.z() << " lastZ=" << it.second.height << " Delta=" << it.second.height - pos.z() << std::endl; //TODO
            //Set the foot for constraints update
            if (it.second.isLeft) needUpdateLeft = true;
            else needUpdateRight = true;
        }
        it.second.height = pos.z();
        //Update contact counters
        if (it.second.isContact && it.second.isLeft) {
            countContactLeft++;
        }
        if (it.second.isContact && !it.second.isLeft) {
            countContactRight++;
        }
    }
    //In strange case when only 3 cleats are
    //contacting per foot, the fourth is added
    for (auto& it : _cleats) {
        if (
            (it.second.isLeft && !it.second.isContact && countContactLeft == 3) ||
            (!it.second.isLeft && !it.second.isContact && countContactRight == 3)
        ) {
            //A collision is added
            it.second.isContact = true;
            it.second.isActive = false;
            //std::cout << "Strange fix for " << it.second.frame << std::endl; //XXX
            if (it.second.isLeft && countDetachingLeft > 0) {
                countDetachingLeft--;
            }
            if (!it.second.isLeft && countDetachingRight > 0) {
                countDetachingRight--;
            }
        }
    }
    //Check for constraint forces
    for (auto& it : _cleats) {
        //Check for negative force 
        if (it.second.isActive && it.second.force < 0.0) {
            //std::cout << "Negative force for " << it.second.frame << std::endl; //XXX
            //Set the foot for constraints update
            if (it.second.isLeft) needUpdateLeft = true;
            else needUpdateRight = true;
        }
    }
    if (countDetachingLeft > 0) {
        needUpdateLeft = true;
    }
    if (countDetachingRight > 0) {
        needUpdateRight = true;
    }

    //If needed, recompute active constraints set
    if (needUpdateLeft) {
        //std::cout << "FINDACTIVE LEFT" << std::endl; //XXX
        findActiveConstraints(dt, true);
    }
    if (needUpdateRight) {
        //std::cout << "FINDACTIVE RIGHT" << std::endl; //XXX
        findActiveConstraints(dt, false);
    }

    //Simulation update
    _simulation.update(dt, _constraints);
    //Assign model position state
    _model.setDOFVect(_simulation.positions());
    
    //Retrieve contact active force
    for (auto& it : _cleats) {
        if (it.second.isActive) {
            it.second.force = _constraints->force[it.second.index];
        }
    }
}

void HumanoidSimulation::printCleatsStatus(bool verbose)
{
    if (verbose) {
        for (const auto& it : _cleats) {
            std::cout 
                << "[" << it.second.frame 
                << ":" << it.second.number 
                << ":" << (it.second.isLeft ? "left" : "right") 
                << "] pos=" << _model.position(it.second.frame, "origin").z()
                << " " << (it.second.isContact ? "isContact" : "")
                << " " << (it.second.isActive ? 
                    "isActive index=" 
                    + std::to_string(it.second.index) 
                    + " force=" 
                    + std::to_string(it.second.force) : "")
                << std::endl;
        }
        if (_constraints != nullptr) {
            std::cout << "Constraint set size=" 
                << _constraints->force.size() << std::endl;
        }
    } else {
        for (const auto& it : _cleats) {
            std::cout 
                << " " << _model.position(it.second.frame, "origin").z()
                << " " << _model.pointVelocity(it.second.frame, "origin", velocities())(5)
                << " " << it.second.force
                << " " << (it.second.isContact ? "1" : "0")
                << " " << (it.second.isActive ? "1" : "0");
        }
        std::cout << std::endl;
    }
}
void HumanoidSimulation::printCleatsStatus(Plot& plot)
{
    VectorLabel vect;
    for (const auto& it : _cleats) {
        vect.append(it.second.frame + ":pos", 
            _model.position(it.second.frame, "origin").z());
        vect.append(it.second.frame + ":vel",
            _model.pointVelocity(it.second.frame, "origin", velocities())(5));
        vect.append(it.second.frame + ":acc",
            _model.pointAcceleration(it.second.frame, "origin", velocities(), accelerations())(5));
        vect.append(it.second.frame + ":force", 
            it.second.force);
        vect.append(it.second.frame + ":isContact", 
            (it.second.isContact ? 1.0 : 0.0));
        vect.append(it.second.frame + ":isActive",
            (it.second.isActive ? 1.0+0.05*it.second.number : 0.0));
    }
    plot.add(vect);
}
        
void HumanoidSimulation::addCleat(const std::string& frame)
{
    _cleats[frame] = {
        frame, 
        std::stoi(frame.substr(frame.find_last_of("_")+1)) - 1,
        _model.frameIndexToBodyId(
            _model.getFrameIndex(frame)),
        (frame.find("left") != std::string::npos), 
        false, false, (size_t)-1, 0.0, 0.0
    };
}
        
void HumanoidSimulation::buildConstraints()
{
    //Clean if necessary
    if (_constraints != nullptr) {
        delete _constraints;
        _constraints = nullptr;
    }
    //ConstraintSet allocation
    _constraints = new RBDL::ConstraintSet();
    //_constraints->SetSolver(RBDLMath::LinearSolverFullPivLU);
    _constraints->SetSolver(RBDLMath::LinearSolverFullPivHouseholderQR);
    //Add all constraints
    size_t indexInSet = 0;
    unsigned int countLeft = 0;
    unsigned int countRight = 0;
    for (auto& it : _cleats) {
        if (it.second.isActive) {
            //Create Z constraint
            //for all active cleat
            _constraints->AddConstraint(
                it.second.bodyId,
                RBDLMath::Vector3d(0.0, 0.0, 0.0),
                RBDLMath::Vector3d(0.0, 0.0, 1.0));
            //Assign constraint index in set
            it.second.index = indexInSet;
            indexInSet++;
            //Count left and right active constraints
            //and assign current (left or right) active
            //count
            unsigned int currentCount;
            if (it.second.isLeft) {
                countLeft++;
                currentCount = countLeft;
            } else {
                countRight++;
                currentCount = countRight;
            }
            //Create X constraints only for the first 
            //two active constraints if existing
            if (currentCount == 1 || currentCount == 2) {
                _constraints->AddConstraint(
                    it.second.bodyId,
                    RBDLMath::Vector3d(0.0, 0.0, 0.0),
                    RBDLMath::Vector3d(1.0, 0.0, 0.0));
                indexInSet++;
            }
            //Create Y constraints only for the first
            //active constraint if existing
            if (currentCount == 1) {
                _constraints->AddConstraint(
                    it.second.bodyId,
                    RBDLMath::Vector3d(0.0, 0.0, 0.0),
                    RBDLMath::Vector3d(0.0, 1.0, 0.0));
                indexInSet++;
            }
            //1 active cleat : 3 foot dof -> 3 constraint
            //2 active cleat : 1 foot dof -> 5 constraint
            //3 active cleat : 0 foot dof -> 6 constraint
        } 
    }
    //Sanity check
    if (countLeft > 3 || countRight > 3) {
        throw std::logic_error(
            "HumanoidSimulation count constraints failed assert: " 
            + std::string(" left=") + std::to_string(countLeft)
            + std::string(" right=") + std::to_string(countRight));
    }

    //Bind and initialize the set 
    //with the RBDL model
    _constraints->Bind(_model.getRBDLModel());
}
        
bool HumanoidSimulation::setActiveCombination(
    const std::vector<size_t>& set, bool isLeft)
{
    //Assign the combination to active constraints
    size_t indexCleat = 0;
    size_t indexComb = 0;
    size_t num1 = -1;
    size_t num2 = -1;
    for (auto& it : _cleats) {
        if (it.second.isLeft == isLeft && it.second.isContact) {
            if (indexComb < set.size() && indexCleat == set[indexComb]) {
                it.second.isActive = true;
                it.second.force = 0.0;
                if (indexComb == 0) {
                    num1 = it.second.number;
                } else if (indexComb == 1) {
                    num2 = it.second.number;
                }
                indexComb++;
            } else {
                it.second.isActive = false;
                it.second.force = 0.0;
            }
            indexCleat++;
        }
    }
    //Check if only foot diagonal cleat are activated
    if (
        set.size() == 2 && 
        ((num1 == 0 && num2 == 2) || (num1 == 1 && num2 == 3))
    ) {
        return true;
    } else {
        return false;
    }
}

void HumanoidSimulation::findActiveConstraints(double dt, bool isLeft)
{
    //Save current simulation state
    ForwardSimulation saveSim = _simulation;
    //Retrieve the number of contacts
    unsigned int countContact = 0;
    for (auto& it : _cleats) {
        if (
            it.second.isContact && 
            it.second.isLeft == isLeft
        ) {
            countContact++;
        }
    }
    //Compute maximum active constraints
    unsigned int maxActive = 
        (countContact >= 3 ? 3 : countContact);

    //Check for no contacting point
    if (countContact == 0) {
        for (auto& it : _cleats) {
            if (it.second.isLeft == isLeft) {
                it.second.isActive = false;
                it.second.force = 0.0;
            }
        }
        //No contact
        return;
    }

    bool bestFound = false;
    double bestForce = 0.0;
    double bestDelta = 0.0;
    std::vector<size_t> bestSet;
    for (int k=maxActive;k>=0;k--) {
        //Iterate over all left or right 
        //cleat combinations
        Combination combination;
        if (k >= 1) {
            combination.startCombination(k, countContact);
        }
        while (true) {
            //Compute next combination
            std::vector<size_t> set;
            if (k >= 1) {
                set = combination.nextCombination();
                if (set.size() == 0) {
                    //No remaining combination
                    break;
                }
            }
            //Assign the combination to active constraints
            bool isDiagonal = setActiveCombination(set, isLeft);
            //Do not activate only the two cleats at foot diagonal
            if (isDiagonal) {
                continue;
            }
            //Assign simulation saved state
            _simulation = saveSim;
            //Rebuildt constraints set
            buildConstraints();
            //Recompute impulses to comply with constraints
            _simulation.computeImpulses(*_constraints);
            //Run the update
            _simulation.update(dt, _constraints);
            //Assign model position state
            _model.setDOFVect(_simulation.positions());
            //Retrieve computed force
            double minForce = +1e10;
            if (k == 0) minForce = 0.0;
            double maxDelta = 0.0;
            //std::cout << "Comb(" << k << "): "; //XXX
            for (auto& it : _cleats) {
                if (it.second.isContact && !it.second.isActive && it.second.isLeft == isLeft) {
                    double height = _model.position(it.second.frame, "origin").z();
                    double delta = it.second.height - height;
                    if (delta > 1e-7) {
                        maxDelta = delta;
                    }
                    /* //XXX
                    std::cout << it.second.number+1 
                        << "-(delta)>" << delta << " ";
                    */
                }
                if (it.second.isActive && it.second.isLeft == isLeft) {
                    it.second.force = _constraints->force(it.second.index);
                    if (minForce > it.second.force) {
                        minForce = it.second.force;
                    }
                    //std::cout << it.second.number+1 << "-(force)>" << it.second.force << " "; //XXX
                }
            }
            /* //XXX
            std::cout << std::endl;
            std::cout << "--> bestForce=" << bestForce << " bestDelta=" << bestDelta << " setSize=" << bestSet.size() << " minForce=" << minForce << " maxDelta=" << maxDelta << std::endl;
            */
            //Get the best possible set using following:
            //- with zero delta
            //- with positive minimum force
            //- with maximal set size
            if (
                !bestFound || 
                (maxDelta < bestDelta) ||
                (maxDelta == 0 && bestForce < 0.0 && minForce > 0.0) ||
                (maxDelta == 0 && bestForce < minForce && (bestForce < 0.0 || bestSet.size() == set.size())) //TODO XXX
            ) {
                //std::cout << "BEST" << std::endl; //XXX
                bestSet = set;
                bestFound = true;
                bestForce = minForce;
                bestDelta = maxDelta;
            }
            if (set.size() == 0) {
                break;
            }
        }
    }
    //Reload state
    _simulation = saveSim;
    if (bestFound) {
        //Assign the combination to active constraints
        setActiveCombination(bestSet, isLeft);
    } else {
        throw std::logic_error(
            "HumanoidSimulation unable to find constraints set");
    }
    //Rebuildt constraints set
    buildConstraints();
    //Recompute impulses to comply with constraints
    _simulation.computeImpulses(*_constraints);
}

}

