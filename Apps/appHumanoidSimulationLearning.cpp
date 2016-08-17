#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <libcmaes/cmaes.h>
#include "Types/MapSeries.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/HumanoidSimulation.hpp"
#include "Plot/Plot.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

/**
 * DOF and base names
 */
static std::vector<std::string> dofsNames = {
    "head_pitch", "head_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
    "left_hip_yaw", "left_hip_pitch", "left_hip_roll",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
    "right_hip_yaw", "right_hip_pitch", "right_hip_roll",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
};
static std::vector<std::string> baseNames = {
    "base_x", "base_y", "base_z",
    "base_roll", "base_pitch", "base_yaw",
};

/**
 * Initialize given HumanoidSimulation with
 * given data at specific time
 */
static void initializeSimulation(
    Leph::HumanoidSimulation& sim, const Leph::MapSeries& data, double t)
{
    //Assign DOFs
    for (const std::string& name : dofsNames) {
        sim.setGoal(name, data.get("read:" + name, t));
        sim.setPos(name, data.get("read:" + name, t));
        sim.setVel(name, 0.0);
    }
    //Assign base
    for (const std::string& name : baseNames) {
        sim.setGoal(name, 0.0);
        sim.setPos(name, 0.0);
        sim.setVel(name, 0.0);
    }
    //Put support foot at origin
    sim.putOnGround();
    sim.putFootAt(0.0, 0.0);
}

/**
 * Assign given model with state at given time
 */
static void assignModelState(
    Leph::HumanoidFixedModel& model, const Leph::MapSeries& series, const std::string& prefix, double t)
{
    //Assign support foot state
    model.setSupportFoot(
        (series.get(prefix + ":is_left_support_foot", t) > 0.5 ? 
        Leph::HumanoidFixedModel::LeftSupportFoot :
        Leph::HumanoidFixedModel::RightSupportFoot));
    //Assign degrees of freedom position
    for (const std::string& name : dofsNames) {
        model.get().setDOF(name, series.get(prefix + ":" + name, t));
    }
    //Assign base state 
    for (const std::string& name : baseNames) {
        model.get().setDOF(name, 0.0);
    }
}

/**
 * Compute the simulation accuraty from
 * given data with given joint model parameters
 */
double scoreSimulation(
    const std::vector<Leph::MapSeries>& container,
    const Eigen::VectorXd& params, bool verbose)
{
    double cost = 0.0;
    Leph::ModelViewer* viewer = nullptr;
    if (verbose) {
        viewer = new Leph::ModelViewer(1200, 900);
    }
    for (size_t i=0;i<container.size();i++) {
        //Retrieve time bound
        double timeStart = container[i].timeMin();
        double timeEnd = container[i].timeMax();
        if (verbose) {
            std::cout << "Log " << i << " start=" << timeStart << " end=" << timeEnd << std::endl;
        }
        //Initialize simulation
        Leph::HumanoidSimulation sim(Leph::SigmabanModel);
        initializeSimulation(sim, container[i], timeStart);
        //Assign joint model parameters
        sim.setJointModelParameters(params);
        //Initialize read model
        Leph::HumanoidFixedModel model(Leph::SigmabanModel);
        assignModelState(model, container[i], "read", timeStart);
        //Iterate over the logged sequence and
        //compute prediction distance
        for (double t=timeStart;t<timeEnd;t+=0.01) {
            //Assign read model state
            assignModelState(model, container[i], "read", t);
            //Assign simulation target
            for (const std::string& name : dofsNames) {
                sim.setGoal(name, container[i].get("goal:" + name, t));
            }
            //Run simulation step
            double step = 0.001;
            for (int k=0;k<(int)(0.01/step);k++) {
                sim.update(step);
            }
            //Enforce support foot constraint
            sim.putOnGround();
            sim.putFootAt(0.0, 0.0);
            //Compute distance
            for (const std::string& name : dofsNames) {
                cost += fabs(model.get().getDOF(name) - sim.getPos(name));
            }
            //Display
            if (verbose) {
                Leph::ModelDraw(model.get(), *viewer);
                Leph::ModelDraw(sim.model(), *viewer);
                viewer->update();
                std::cout << t << " " << cost << std::endl;
            }
        }
    }
    if (verbose) {
        delete viewer;
    }
    return cost;
}

int main(int argc, char** argv)
{
    //Retrieve data log paths
    std::vector<std::string> filenames;
    for (size_t i=1;i<(size_t)argc;i++) {
        filenames.push_back(argv[i]);
    }

    //Load data into MapSeries
    std::vector<Leph::MapSeries> container;
    for (size_t i=0;i<filenames.size();i++) {
        //Load data log
        container.push_back(Leph::MapSeries());
        container.back().importData(filenames[i]);
        //Print statistics
        std::cout << "Loaded " 
            << filenames[i] << ": "
            << container.back().dimension() << " series from " 
            << container.back().timeMin() << "s to " 
            << container.back().timeMax() << "s with length "
            << container.back().timeMax()-container.back().timeMin() << "s" << std::endl;
    }

    //Initialize the joint model parameters
    Eigen::VectorXd initParams(8);
    //Friction velocity limit
    initParams(0) = 0.2;
    //Friction viscous
    initParams(1) = 0.01;
    //Friction static Coulomb
    initParams(2) = 1.0;
    //Friction static breakaway
    initParams(3) = 1.0;
    //Control proportional gain
    initParams(4) = 40.0;
    //Control max torque at zero velocity
    initParams(5) = 40.0;
    //Control max velocity at zero torque
    initParams(6) = 50.0;
    //Control lag in seconds
    initParams(7) = 0.00;
    
    /*
    initParams << 
        0.194646717515798, 
        0.0323399568316223,   
        0.98449335186818,    
        0.9402908767844,   
        39.8579634954168,
        40.0668258205848,
        50.078570860388,
        0.0492262861526356;
    //Test initial paramsters
    scoreSimulation(container, initParams, true);
    return 0;
    */

    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&container](const Eigen::VectorXd& params) 
    {
        //Bound positive parameters
        Eigen::VectorXd tmpParams = params;
        double cost = 0.0;
        for (size_t i=0;i<(size_t)params.size();i++) {
            if (tmpParams(i) < 0.0) {
                cost += 1000.0 - 1000.0*tmpParams(i);
                tmpParams(i) = 0.0;
            }
        }
        //Score simulation
        try {
            cost += scoreSimulation(container, params, false);
        } catch (const std::runtime_error& e) {
            cost += 10000.0;
        }
        return cost;
    };
    
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(initParams, -1.0, 10);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(true);
    cmaparams.set_restarts(1);
    cmaparams.set_max_iter(100);
    
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams);
    
    //Retrieve best Trajectories and score
    Eigen::VectorXd bestParams = cmasols.get_best_seen_candidate().get_x_dvec();
    double bestScore = cmasols.get_best_seen_candidate().get_fvalue();
    std::cout << "BestScore: " << bestScore << std::endl;
    std::cout << "BestParams: " << bestParams.transpose() << std::endl;

    return 0;
}

