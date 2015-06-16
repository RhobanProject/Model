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
    inv.addDOF("right_hip_yaw");
    inv.addDOF("right_hip_pitch");
    inv.addDOF("right_hip_roll");
    inv.addDOF("right_knee");
    inv.addDOF("right_ankle_pitch");
    inv.addDOF("right_ankle_roll");
    inv.addDOF("left_hip_yaw");
    inv.addDOF("left_hip_pitch");
    inv.addDOF("left_hip_roll");
    inv.addDOF("left_knee");
    inv.addDOF("left_ankle_pitch");
    inv.addDOF("left_ankle_roll");
    
    //Add degree of freedom limit bounds
    inv.setLowerBound("right_hip_yaw", -M_PI/4);
    inv.setUpperBound("right_hip_yaw", M_PI/4);
    inv.setLowerBound("right_hip_pitch", -M_PI/3);
    inv.setUpperBound("right_hip_pitch", M_PI/3);
    inv.setLowerBound("right_hip_roll", -M_PI/4);
    inv.setUpperBound("right_hip_roll", M_PI/4);
    inv.setLowerBound("right_knee", 0.0);
    inv.setUpperBound("right_knee", M_PI/2);
    inv.setLowerBound("right_ankle_pitch", -M_PI/2);
    inv.setUpperBound("right_ankle_pitch", M_PI/2);
    inv.setLowerBound("right_ankle_roll", -M_PI/2);
    inv.setUpperBound("right_ankle_roll", M_PI/2);
    inv.setLowerBound("left_hip_yaw", -M_PI/4);
    inv.setUpperBound("left_hip_yaw", M_PI/4);
    inv.setLowerBound("left_hip_pitch", -M_PI/3);
    inv.setUpperBound("left_hip_pitch", M_PI/3);
    inv.setLowerBound("left_hip_roll", -M_PI/4);
    inv.setUpperBound("left_hip_roll", M_PI/4);
    inv.setLowerBound("left_knee", 0.0);
    inv.setUpperBound("left_knee", M_PI/2);
    inv.setLowerBound("left_ankle_pitch", -M_PI/2);
    inv.setUpperBound("left_ankle_pitch", M_PI/2);
    inv.setLowerBound("left_ankle_roll", -M_PI/2);
    inv.setUpperBound("left_ankle_roll", M_PI/2);
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
    inv.addTargetOrientation("flying_foot", "right_foot_tip");
    
    return inv;
}
Leph::InverseKinematics doubleSupportPhaseModel(Leph::Model& model)
{
    Leph::InverseKinematics inv(model);
    initDOF(inv);

    //Set Center of Mass target position
    inv.addTargetCOM();
    //Set flying foot position and orientation constraints
    inv.addTargetOrientation("flying_foot", "right_foot_tip");
    inv.addTargetPosition("flying_foot", "right_foot_tip");

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
        tau(model.getDOFIndex("base_x")) = 0.0;
        tau(model.getDOFIndex("base_y")) = 0.0;
        tau(model.getDOFIndex("base_z")) = 0.0;
        tau(model.getDOFIndex("base_yaw")) = 0.0;
        tau(model.getDOFIndex("base_pitch")) = 0.0;
        tau(model.getDOFIndex("base_roll")) = 0.0;
        //Compute infinity norm and euclidian norm 
        value += 50.0*tau.lpNorm<Eigen::Infinity>();
        value += tau.norm();
        //Compute model target error
        Eigen::VectorXd fvect(inv.values());
        inv.operator()(y, fvect);
        value += 100.0*fvect.norm();
        //Compute flying foot constraint
        Eigen::Vector3d pos = 
            model.position("right_foot_tip", "origin");
        if (pos.z() < 0.05) {
            value += 10.0 + 10.0*(0.05-pos.z());
        }
        if (pos.y() > -2.0*0.039995) {
            value += 10.0 + 10.0*(pos.y()+2.0*0.039995);
        }
        //Compute limit constraint
        for (size_t i=0;i<(size_t)y.size();i++) {
            if (y(i) < -M_PI/3.0 || y(i) > M_PI/3.0) {
                value += 10.0;
            }
        }
        if (model.getDOF("left_knee") < 0.0) {
            value += 10.0 - 10.0*model.getDOF("left_knee");
        }
        if (model.getDOFIndex("right_knee") < 0.0) {
            value += 10.0 - 10.0*model.getDOF("right_knee");
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
    tau(model.getDOFIndex("base_x")) = 0.0;
    tau(model.getDOFIndex("base_y")) = 0.0;
    tau(model.getDOFIndex("base_z")) = 0.0;
    tau(model.getDOFIndex("base_yaw")) = 0.0;
    tau(model.getDOFIndex("base_pitch")) = 0.0;
    tau(model.getDOFIndex("base_roll")) = 0.0;
    for (size_t i=0;i<(size_t)tau.size();i++) {
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

