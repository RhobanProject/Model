#include <iostream>
#include <Eigen/Dense>
#include <libcmaes/cmaes.h>
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/InverseKinematics.hpp"
#include "Model/NullSpace.hpp"

void exploreCenterOfMassPose(
    const Eigen::Vector3d& com,
    Leph::InverseKinematics& inv)
{
    /*
    std::vector<Eigen::VectorXd> exploredContainer;
    
    //Initial convergence
    inv.run(0.0001, 100);
    
    //Explore and discretize the Nullspace
    Leph::NullSpace nullspace(inv);
    nullspace.exploreKernelDiscretized(
        inv.getDOFSubset(), exploredContainer, 0.1, 20000, false); 
    */
}

/**
 * Declare degrees of freedom and set bounds 
 * to given InverseKinematics
 */
void initDOF(Leph::InverseKinematics& inv)
{
    //Add degrees of freedom subset
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
}

/**
 * Initialize and return InverseKinematics instance
 * constrainted for single and double support phase
 */
Leph::InverseKinematics singleSupportPhaseModel(Leph::Model& model)
{
    Leph::InverseKinematics inv(model);
    initDOF(inv);
    
    //Set Center of Mass target position
    inv.addTargetCOM();
    //Set flying foot orientation constraints
    inv.addTargetOrientation("flying foot", "right foot tip");
    
    return inv;
}
Leph::InverseKinematics doubleSupportPhaseModel(Leph::Model& model)
{
    Leph::InverseKinematics inv(model);
    initDOF(inv);

    //Set Center of Mass target position
    inv.addTargetCOM();
    //Set flying foot position and orientation constraints
    inv.addTargetOrientation("flying foot", "right foot tip");
    inv.addTargetPosition("flying foot", "right foot tip");

    return inv;
}

/**
 * Find and return target single support phase pose
 */
Eigen::VectorXd findTargetPose(Leph::Model& model)
{
    //Initializing constrained models
    Leph::InverseKinematics inv = 
        singleSupportPhaseModel(model);
    //inv.addTargetOrientation("trunk", "trunk");
    inv.targetCOM().x() = 0.0;
    inv.targetCOM().y() = 0.0;
    
    //Try to find minimum torques configuration with
    //CMA-ES optimization
    //Use infinity norm minimization
    libcmaes::FitFunc fitness = [&model, &inv]
        (const double *x, const int N) 
    {
        double value = 0.0;
        //Conversion to Eigen vector
        Eigen::VectorXd y(N);
        for (int i=0;i<N;i++) {
            y(i) = x[i];
        }
        //Compute pose torques
        inv.setDOFSubset(y);
        Eigen::VectorXd tau = model.inverseDynamics();
        //Set base torques to zero
        tau(model.getDOFIndex("base Tx")) = 0.0;
        tau(model.getDOFIndex("base Ty")) = 0.0;
        tau(model.getDOFIndex("base Tz")) = 0.0;
        tau(model.getDOFIndex("base yaw")) = 0.0;
        tau(model.getDOFIndex("base pitch")) = 0.0;
        tau(model.getDOFIndex("base roll")) = 0.0;
        //Compute infinity norm and euclidian norm 
        value += 50.0*tau.lpNorm<Eigen::Infinity>();
        value += tau.norm();
        //Compute model target error
        Eigen::VectorXd fvect(inv.values());
        inv.operator()(y, fvect);
        value += 100.0*fvect.norm();
        //Compute flying foot constraint
        Eigen::Vector3d pos = 
            model.position("right foot tip", "origin");
        if (pos.z() < 0.05) {
            value += 10.0 + 10.0*(0.05-pos.z());
        }
        if (pos.y() > -2.0*0.039995) {
            value += 10.0 + 10.0*(pos.y()+2.0*0.039995);
        }
        //Compute limit constraint
        for (size_t i=0;i<y.size();i++) {
            if (y(i) < -M_PI/3.0 || y(i) > M_PI/3.0) {
                value += 10.0;
            }
        }
        if (model.getDOF("left knee") < 0.0) {
            value += 10.0 - 10.0*model.getDOF("left knee");
        }
        if (model.getDOFIndex("right knee") < 0.0) {
            value += 10.0 - 10.0*model.getDOF("right knee");
        }

        return value;
    };
    libcmaes::CMASolutions cmasols;
    do {
        //Starting point
        std::vector<double> x0(inv.getDOFSubset().size(), 0.0);
        //Optimization configuration
        double sigma = 0.1;
        libcmaes::CMAParameters<> cmaparams(x0, sigma);
        cmaparams.set_quiet(false);
        cmaparams.set_mt_feval(false);
        cmaparams.set_str_algo("bibop");
        //cmaparams.set_str_algo("acmaes");
        //cmaparams.set_max_iter(1000);
        //Run optimization
        cmasols = libcmaes::cmaes<>(fitness, cmaparams);
        //Retrieve solution
        std::cout << "best solution: " << cmasols << std::endl;
        std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
    } while (false && cmasols.best_candidate().get_fvalue() >= 10.0);
    inv.setDOFSubset(cmasols.best_candidate().get_x_dvec());
    Eigen::VectorXd tau = model.inverseDynamics();
    tau(model.getDOFIndex("base Tx")) = 0.0;
    tau(model.getDOFIndex("base Ty")) = 0.0;
    tau(model.getDOFIndex("base Tz")) = 0.0;
    tau(model.getDOFIndex("base yaw")) = 0.0;
    tau(model.getDOFIndex("base pitch")) = 0.0;
    tau(model.getDOFIndex("base roll")) = 0.0;
    for (size_t i=0;i<tau.size();i++) {
        std::cout << model.getDOFName(i) 
            << ": " << tau(i) 
            << " pos=" << model.getDOF(model.getDOFName(i)) << std::endl;
    }
    std::cout << "TorquesSumMax: " 
        << tau.lpNorm<Eigen::Infinity>() << std::endl;
    std::cout << "TorquesSumNorm: " 
        << tau.norm() << std::endl;
    std::cout << "COM: " << model.centerOfMass("origin").transpose() << std::endl;

    return cmasols.best_candidate().get_x_dvec();
}

int main()
{
    //Sigmaban foot fixed model
    Leph::HumanoidFixedModel model(Leph::SigmabanModel);
    model.setSupportFoot(
        Leph::HumanoidFixedModel::LeftSupportFoot);
    
    //Initializing constrained models
    Leph::InverseKinematics invSingleSupport = 
        singleSupportPhaseModel(model.get());
    Leph::InverseKinematics invDoubleSupport = 
        doubleSupportPhaseModel(model.get());

    //Generate target single support phase 
    //degree of freedom subset pose
    Eigen::VectorXd targetPose = findTargetPose(model.get());
    
    //Viewer
    Leph::ModelViewer viewer(1200, 900);
    //Display best found pose
    invSingleSupport.setDOFSubset(targetPose);
    while (viewer.update()) {
        Leph::ModelDraw(model.get(), viewer);
    }

    return 0;
}

