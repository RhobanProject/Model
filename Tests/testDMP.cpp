#include <iostream>
#include <libcmaes/cmaes.h>
#include "DMP/DMP.hpp"
#include "Plot/Plot.hpp"

static void testLearning()
{
    Leph::SmoothSpline target;
    target.addPoint(0.0, 0.0, 0.0, 0.0);
    target.addPoint(1.0, 0.5, 0.5, -0.6);
    target.addPoint(2.0, -0.5, 0.4, 0.7);
    target.addPoint(3.0, 1.0, 0.0, 0.0);
    target.plot().plot("time", "all").render();

    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&target](const Eigen::VectorXd& params) 
    {
        Leph::DMP dmp(1, 5);
        dmp.setParameters(params, true, false, true);
        Eigen::VectorXd startPos(1);
        Eigen::VectorXd startVel(1);
        Eigen::VectorXd startAcc(1);
        Eigen::VectorXd endPos(1);
        Eigen::VectorXd endVel(1);
        Eigen::VectorXd endAcc(1);
        startPos(0) = target.pos(0);
        startVel(0) = target.vel(0);
        startAcc(0) = target.acc(0);
        endPos(0) = target.pos(target.max());
        endVel(0) = target.vel(target.max());
        endAcc(0) = target.acc(target.max());
        dmp.init(
            target.max(), 
            startPos, startVel, startAcc,
            endPos, endVel, endAcc);
        double cost = 0.0;
        while (dmp.currentTime() <= target.max()) {
            cost += pow(target.pos(dmp.currentTime()) - dmp.statePos()(0), 2);
            dmp.step(0.01);
        }

        return cost;
    };
    
    //CMAES initialization
    Leph::DMP dmp(1, 5);
    Eigen::VectorXd params = dmp.getParameters(true, false, true);
    libcmaes::CMAParameters<> cmaparams(
        params, -1.0, 10);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(true);
    cmaparams.set_restarts(4);
    cmaparams.set_max_iter(5000);

    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams);
    
    //Retrieve best Trajectories and score
    params = cmasols.get_best_seen_candidate().get_x_dvec();
    double score = 
        cmasols.get_best_seen_candidate().get_fvalue();
    std::cout << "BestScore: " << score << std::endl;

    dmp.setParameters(params, true, false, true);
    Eigen::VectorXd startPos(1);
    Eigen::VectorXd startVel(1);
    Eigen::VectorXd startAcc(1);
    Eigen::VectorXd endPos(1);
    Eigen::VectorXd endVel(1);
    Eigen::VectorXd endAcc(1);
    startPos(0) = target.pos(0);
    startVel(0) = target.vel(0);
    startAcc(0) = target.acc(0);
    endPos(0) = target.pos(target.max());
    endVel(0) = target.vel(target.max());
    endAcc(0) = target.acc(target.max());
    Leph::Plot plot;
    dmp.init(
        target.max(), 
        startPos, startVel, startAcc,
        endPos, endVel, endAcc);
    while (dmp.currentTime() <= target.max()) {
        plot.add(Leph::VectorLabel(
            "t", dmp.currentTime(),
            "fitted", dmp.statePos()(0),
            "target", target.pos(dmp.currentTime())
        ));
        dmp.step(0.01);
    }
    dmp.init(
        target.max(), 
        startPos, startVel, startAcc,
        2.0*endPos, endVel, endAcc);
    while (dmp.currentTime() <= target.max()) {
        plot.add(Leph::VectorLabel(
            "t", dmp.currentTime(),
            "fitted", dmp.statePos()(0)
        ));
        dmp.step(0.01);
    }
    dmp.init(
        target.max(), 
        startPos, startVel, startAcc,
        -2.0*endPos, endVel, endAcc);
    while (dmp.currentTime() <= target.max()) {
        plot.add(Leph::VectorLabel(
            "t", dmp.currentTime(),
            "fitted", dmp.statePos()(0)
        ));
        dmp.step(0.01);
    }
    plot.plot("t", "all").render();
}

int main()
{
    Leph::Plot plot;

    unsigned int dim = 1;
    unsigned int num = 5;
    Leph::DMP dmp(dim, num);
    double timeLength = 2.0;
    Eigen::VectorXd startPos = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd startVel = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd startAcc = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd endPos = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd endVel = Eigen::VectorXd::Zero(dim);
    Eigen::VectorXd endAcc = Eigen::VectorXd::Zero(dim);
    startPos(0) = 0.0;
    startVel(0) = 1.0;
    startAcc(0) = 3.0;
    endPos(0) = 2.0;
    endVel(0) = -1.0;
    endAcc(0) = 0.0;

    dmp.init(
        timeLength, 
        startPos, startVel, startAcc, 
        endPos, endVel, endAcc);
    for (size_t i=0;i<num;i++) {
        std::cout << "Kernel " << i 
            << ": center=" << dmp.kernelCenter(i) 
            << " width=" << dmp.kernelWidth(i) << std::endl;
    }
    while (true) {
        if (dmp.currentTime() >= timeLength) {
            break;
        }
        dmp.step(0.01);
        plot.add(Leph::VectorLabel(
            "t", dmp.currentTime(),
            "phase", dmp.statePhase(),
            "gating", dmp.stateGating(),
            "forcing", dmp.forcingFunction(dmp.statePhase(), dmp.stateGating())(0),
            "pos", dmp.statePos()(0),
            "vel", dmp.stateVel()(0),
            "acc", dmp.stateAcc()(0)
        ));
    }
    plot
        .plot("t", "all")
        .render();
    
    //Test DMP fitting
    testLearning();

    return 0;
}

