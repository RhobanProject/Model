#include <iostream>
#include <algorithm>
#include <libcmaes/cmaes.h>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/SigmabanFixedModel.hpp"
#include "Model/InverseKinematics.hpp"
#include "Model/NullSpace.hpp"

int main()
{
    //Sigmaban foot fixed model
    Leph::SigmabanFixedModel model;
    model.setSupportFoot(
        Leph::SigmabanFixedModel::LeftSupportFoot);

    //Viewer
    Leph::ModelViewer viewer(1200, 900);
    
    //Add degree of freedom subset
    Leph::InverseKinematics inv(model.get());
    inv.addDOF("right hip yaw");
    inv.addDOF("right hip pitch");
    inv.addDOF("right hip roll");
    inv.addDOF("right knee");
    inv.addDOF("right foot pitch");
    inv.addDOF("right foot roll");
    inv.addDOF("left hip yaw");
    inv.addDOF("left hip pitch");
    inv.addDOF("left hip roll");
    inv.addDOF("left knee");
    inv.addDOF("left foot pitch");
    inv.addDOF("left foot roll");

    //Add degree of freedom limit bounds
    inv.setLowerBound("right hip yaw", -M_PI/4);
    inv.setUpperBound("right hip yaw", M_PI/4);
    inv.setLowerBound("right hip pitch", -M_PI/3);
    inv.setUpperBound("right hip pitch", M_PI/3);
    inv.setLowerBound("right hip roll", -M_PI/4);
    inv.setUpperBound("right hip roll", M_PI/4);
    inv.setLowerBound("right knee", 0.0);
    inv.setUpperBound("right knee", M_PI/2);
    inv.setLowerBound("right foot pitch", -M_PI/2);
    inv.setUpperBound("right foot pitch", M_PI/2);
    inv.setLowerBound("right foot roll", -M_PI/2);
    inv.setUpperBound("right foot roll", M_PI/2);
    inv.setLowerBound("left hip yaw", -M_PI/4);
    inv.setUpperBound("left hip yaw", M_PI/4);
    inv.setLowerBound("left hip pitch", -M_PI/3);
    inv.setUpperBound("left hip pitch", M_PI/3);
    inv.setLowerBound("left hip roll", -M_PI/4);
    inv.setUpperBound("left hip roll", M_PI/4);
    inv.setLowerBound("left knee", 0.0);
    inv.setUpperBound("left knee", M_PI/2);
    inv.setLowerBound("left foot pitch", -M_PI/2);
    inv.setUpperBound("left foot pitch", M_PI/2);
    inv.setLowerBound("left foot roll", -M_PI/2);
    inv.setUpperBound("left foot roll", M_PI/2);
    
    //Set Center of Mass target position
    inv.addTargetCOM();
    inv.targetCOM().z() -= 0.04;
    //Set flying foot constraints
    inv.addTargetOrientation("flying foot", "right foot tip");
    inv.addTargetPosition("flying foot", "right foot tip");
        
    //Initial convergence
    inv.run(0.0001, 100);

    //Explore and discretize the Nullspace
    std::vector<Eigen::VectorXd> exploredContainer;
    Leph::NullSpace nullspace(inv);
    nullspace.exploreKernelDiscretized(
        inv.getDOFSubset(), exploredContainer, 0.1, 20000, false); 

    //Sort explored state by minimum torques
    std::sort(exploredContainer.begin(), exploredContainer.end(), 
        [&inv, &model]
        (const Eigen::VectorXd& state1, const Eigen::VectorXd& state2) {
            inv.setDOFSubset(state1);
            double tmp1 = model.get()
                .inverseDynamicsClosedLoop("right foot tip")
                .lpNorm<Eigen::Infinity>();
            inv.setDOFSubset(state2);
            double tmp2 = model.get()
                .inverseDynamicsClosedLoop("right foot tip")
                .lpNorm<Eigen::Infinity>();
            return tmp1 < tmp2;
        });

    //Display all found null space discretized states
    size_t index = 0;
    double t = 0;
    while (viewer.update()) {
        t += 0.02;
        inv.setDOFSubset(exploredContainer[index]);
        
        Leph::ModelDraw(model.get(), viewer);
        
        std::cout << "Index=" << index 
            << "/" << exploredContainer.size()-1 
            << " --- COM: " << model.get()
            .centerOfMass("origin").transpose() << std::endl;
        Eigen::VectorXd tau = model.get().inverseDynamicsClosedLoop(
            model.get().getFrameIndex("right foot tip"));
        std::cout << "TorquesSumMax: " 
            << tau.lpNorm<Eigen::Infinity>() << std::endl;
        std::cout << "TorquesSumNorm: " 
            << tau.norm() << std::endl;

        if (t > 1.0) {
            index++;
            t = 0.0;
        }
        if (index >= exploredContainer.size()) {
            index = 0;
        }
    }

    //Try to find minimum torques configuration with
    //CMA-ES optimization
    //Use infinity norm minimization
    libcmaes::FitFunc fitness = [&model, &inv, &nullspace]
        (const double *x, const int N) 
    {
        double value = 0.0;
        Eigen::VectorXd y(N);
        for (int i=0;i<N;i++) {
            y(i) = x[i];
        }
        inv.setDOFSubset(y);
        if (!nullspace.checkConstraints(y)) {
            value += 1000.0;
        }
        value += model.get()
            .inverseDynamicsClosedLoop("right foot tip", true)
            .lpNorm<Eigen::Infinity>();
        Eigen::VectorXd fvect(inv.values());
        inv.operator()(y, fvect);
        value += 100.0*fvect.norm();
        return value;
    };
    //Starting point
    std::vector<double> x0(inv.getDOFSubset().size(), 0.0);
    for (int i=0;i<inv.getDOFSubset().size();i++) {
        x0[i] = inv.getDOFSubset()(i);
    }
    //Optimization init
    double sigma = 0.1;
    libcmaes::CMAParameters<> cmaparams(x0, sigma);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(false);
    cmaparams.set_str_algo("acmaes");
    cmaparams.set_max_iter(1000);
    //Run optimization
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
    std::cout << "best solution: " << cmasols << std::endl;
    std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
    Eigen::VectorXd tau = model.get().inverseDynamicsClosedLoop(
        model.get().getFrameIndex("right foot tip"), true);
    std::cout << "TorquesSumMax: " 
        << tau.lpNorm<Eigen::Infinity>() << std::endl;
    std::cout << "TorquesSumNorm: " 
        << tau.norm() << std::endl;
    
    //Display best found CMA-ES torques pose
    inv.setDOFSubset(cmasols.best_candidate().get_x_dvec());
    while (viewer.update()) {
        Leph::ModelDraw(model.get(), viewer);
    }
    
    return 0;
}

