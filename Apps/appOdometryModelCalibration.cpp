#include <iostream>
#include <fstream>
#include <string>
#include <libcmaes/cmaes.h>
#include "Plot/Plot.hpp"
#include "Model/HumanoidFixedModel.hpp"
#include "Model/OdometryModel.hpp"

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "./app odometry_log_filepath" << std::endl;
        return 1;
    }
    Leph::OdometryModel::OdometryModelType type = 
        Leph::OdometryModel::CorrectionLinear;

    //Load data from log
    std::vector<std::vector<Eigen::Vector3d>> readTrajsPose;
    std::vector<std::vector<Leph::HumanoidFixedModel::SupportFoot>> readTrajsSupport;
    std::vector<Eigen::Vector2d> targetDisplacements;
    std::string filename(argv[1]);
    std::ifstream file(filename);
    std::cout << "Loading data from " << filename << std::endl;
    size_t lastSeq = -1;
    while (file.good() && file.peek() != EOF) {
        while (file.peek() == ' ' || file.peek() == '\n') {
            file.ignore();
        }
        size_t seq;
        size_t index;
        double poseX;
        double poseY;
        double poseYaw;
        int supportFoot;
        double targetX;
        double targetY;
        double targetZ;
        file >> seq;
        file >> index;
        file >> poseX;
        file >> poseY;
        file >> poseYaw;
        file >> supportFoot;
        file >> targetX;
        file >> targetY;
        file >> targetZ;
        if (lastSeq != seq) {
            readTrajsPose.push_back(std::vector<Eigen::Vector3d>());
            readTrajsSupport.push_back(std::vector<Leph::HumanoidFixedModel::SupportFoot>());
            targetDisplacements.push_back(Eigen::Vector2d(targetX, targetY));
        }
        lastSeq = seq;
        readTrajsPose.back().push_back(Eigen::Vector3d(poseX, poseY, poseYaw));
        readTrajsSupport.back().push_back((Leph::HumanoidFixedModel::SupportFoot)supportFoot);
    }
    file.close();
    std::cout << "Loaded " << readTrajsPose.size() << " sequences" << std::endl;
    for (size_t i=0;i<readTrajsPose.size();i++) {
        std::cout << "Seq " << i << " with " << readTrajsPose[i].size() << " points " 
            << "Displacement: " << targetDisplacements[i].transpose() << std::endl;
    }
    
    //Fitness function
    libcmaes::FitFuncEigen fitness = 
        [&type, &readTrajsPose, &readTrajsSupport, &targetDisplacements](const Eigen::VectorXd& params) 
        {
            //Check bounds
            if (params.lpNorm<Eigen::Infinity>() > 10.0) {
                return 1000.0 + 1000.0*params.lpNorm<Eigen::Infinity>();
            }
            double error = 0.0;
            int count = 0;
            for (size_t i=0;i<readTrajsPose.size();i++) {
                Leph::OdometryModel odometry(type);
                odometry.parameters() = params;
                odometry.reset();
                for (size_t j=0;j<readTrajsPose[i].size();j++) {
                    odometry.update(
                        readTrajsPose[i][j], 
                        readTrajsSupport[i][j]);
                }
                error += (
                    targetDisplacements[i] 
                    - odometry.state().segment(0, 2)
                ).squaredNorm();
                count++;
            }
            if (count == 0) {
                return 0.0;
            } else {
                return sqrt(error/count);
            }
        };
    Leph::OdometryModel odometry(type);
    Eigen::VectorXd initParams = odometry.parameters();
    //CMAES initialization
    libcmaes::CMAParameters<> cmaparams(
        initParams, -1.0, 10);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(false);
    cmaparams.set_str_algo("abipop");
    cmaparams.set_elitism(true);
    cmaparams.set_restarts(2);
    cmaparams.set_max_iter(1000);
    //Run optimization
    libcmaes::CMASolutions cmasols = 
        libcmaes::cmaes<>(fitness, cmaparams);
    //Retrieve best Trajectories and score
    Eigen::VectorXd bestParams = 
        cmasols.get_best_seen_candidate().get_x_dvec();
    double rmse = cmasols.get_best_seen_candidate().get_fvalue();
    std::cout << "==========================" << std::endl;
    std::cout << "RMSE: " << rmse << std::endl; 
    std::cout << "CALIBRATION RESULT: " << std::endl << bestParams << std::endl;

    return 0;
}

