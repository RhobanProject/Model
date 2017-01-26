#include <iostream>
#include <string>
#include "Model/HumanoidSimulation.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/JointModel.hpp"
#include "TrajectoryGeneration/TrajectoryUtils.h"
#include "Utils/AxisAngle.h"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Types/MapSeries.hpp"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

/**
 * Run HumanoidSimulation
 * on given log or splines
 */
int main(int argc, char** argv) 
{
    //Parse arguments
    if (
        argc != 3 || (
        argv[1] != std::string("LOG") && 
        argv[1] != std::string("LOG_CYCLE") && 
        argv[1] != std::string("TRAJ_CYCLE") && 
        argv[1] != std::string("TRAJ"))
    ) {
        std::cout << "Usage: ./app LOG filepath.mapseries" << std::endl;
        std::cout << "Usage: ./app TRAJ filepath.splines" << std::endl;
        std::cout << "Usage: ./app LOG_CYCLE filepath.mapseries" << std::endl;
        std::cout << "Usage: ./app TRAJ_CYCLE filepath.splines" << std::endl;
        return 1;
    }
    std::string filename = argv[2];
    bool isTrajMode;
    bool isCycleMode;
    if (argv[1] == std::string("TRAJ")) {
        isTrajMode = true;
        isCycleMode = false;
        std::cout << "Reading episodic Traj: " << filename << std::endl;
    } else if (argv[1] == std::string("LOG")) {
        isTrajMode = false;
        isCycleMode = false;
        std::cout << "Reading episodic Log: " << filename << std::endl;
    } else if (argv[1] == std::string("TRAJ_CYCLE")) {
        isTrajMode = true;
        isCycleMode = true;
        std::cout << "Reading periodic Traj: " << filename << std::endl;
    } else if (argv[1] == std::string("LOG_CYCLE")) {
        isTrajMode = false;
        isCycleMode = true;
        std::cout << "Reading periodic Log: " << filename << std::endl;
    } else {
        return 0;
    }

    //Viewer initialization
    Leph::ModelViewer viewer(1200, 900);
    double timeMin = 0.0;
    double timeMax = 0.0;
    
    //Load data from trajectories
    Leph::Trajectories trajs;
    if (isTrajMode) {
        trajs.importData(filename);
        timeMin = trajs.min();
        timeMax = trajs.max();
        std::cout << "Loading trajs "
            << filename << ": "
            << "from " << trajs.min() 
            << "s to " << trajs.max() 
            << "s with length " 
            << trajs.max()-trajs.min() 
            << "s" << std::endl;
    }

    //Load data into MapSeries
    Leph::MapSeries logs;
    if (!isTrajMode) {
        logs.importData(filename);
        timeMin = logs.timeMin();
        timeMax = logs.timeMax();
        std::cout << "Loading log " 
            << filename << ": "
            << logs.dimension() << " series from " 
            << logs.timeMin() << "s to " 
            << logs.timeMax() << "s with length "
            << logs.timeMax()-logs.timeMin() << "s" << std::endl;
    }

    //Add begin and end waits 
    //in non cycle mode
    if (!isCycleMode) {
        timeMin -= 0.5;
        timeMax += 2.0;
    }
        
    //Main loop
    Leph::Plot plot;
    bool isContinue = true;
    bool isInitialized = false;
    while (isContinue) {
        //Simulator and goal model initialization
        Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);
        Leph::HumanoidSimulation sim(Leph::SigmabanModel);
        //No reinitialition in cycle mode
        if (!isCycleMode) {
            isInitialized = false;
        }
        //Simulation loop
        for (double t=timeMin;t<timeMax;t+=0.01) {
            //Expected velocities and accelerations
            Eigen::VectorXd dq;
            Eigen::VectorXd ddq;
            if (isTrajMode) {
                //Compute DOF target and velocities from Cartesian
                bool isSuccess = Leph::TrajectoriesComputeKinematics(
                    t, trajs, modelGoal, dq, ddq);
                if (!isSuccess) {
                    std::cout << "IK Error t=" << t << std::endl;
                    exit(1);
                }
                //Assign simulation DOF goal
                for (const std::string& name : Leph::NamesDOF) {
                    sim.setGoal(name, modelGoal.get().getDOF(name));
                }
            } else {
                //Assign simulation DOF goal
                for (const std::string& name : Leph::NamesDOF) {
                    sim.setGoal(name, logs.get("goal:" + name, t));
                    modelGoal.get().setDOF(name, logs.get("goal:" + name, t));
                }
            } 
            //State Initialization
            if (!isInitialized) {
                for (const std::string& name : Leph::NamesDOF) {
                    sim.setGoal(name, modelGoal.get().getDOF(name));
                    sim.setPos(name, modelGoal.get().getDOF(name));
                    if (isTrajMode) {
                        size_t indexModel = modelGoal.get().getDOFIndex(name);
                        sim.setVel(name, dq(indexModel));
                    } else {
                        sim.setVel(name, 0.0);
                    }
                    //Reset backlash state
                    sim.jointModel(name).resetBacklashState();
                }
                for (const std::string& name : Leph::NamesBase) {
                    //TODO XXX base == trunk vel initialization. Careful euler angle ...
                    sim.setVel(name, 0.0); 
                }
                sim.putOnGround(
                    Leph::HumanoidFixedModel::LeftSupportFoot);
                sim.putFootAt(0.0, 0.0,
                    Leph::HumanoidFixedModel::LeftSupportFoot);
                isInitialized = true;
            }
            //Run simulation
            for (int k=0;k<10;k++) {
                sim.update(0.001);
            }
            //Display
            Leph::ModelDraw(modelGoal.get(), viewer, 0.3);
            Leph::ModelDraw(sim.model(), viewer);
            Leph::CleatsDraw(sim, viewer);
            //Viewer update
            if (!viewer.update()) {
                isContinue = false;
                break;
            }
        }
    }

    return 0;
}

