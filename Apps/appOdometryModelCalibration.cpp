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
        Leph::OdometryModel::CorrectionLinearWithAzimuth;

    //Load data from log
    std::vector<std::vector<Eigen::Vector3d>> readTrajsPose;
    std::vector<std::vector<Leph::HumanoidFixedModel::SupportFoot>> readTrajsSupport;
    std::vector<std::vector<Eigen::Vector3d>> goalTrajsPose;
    std::vector<std::vector<Leph::HumanoidFixedModel::SupportFoot>> goalTrajsSupport;
    std::vector<std::vector<Eigen::Vector4d>> walkTrajsOrder;
    std::vector<std::vector<double>> walkTrajsPhase;
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
        double readPoseX;
        double readPoseY;
        double readPoseYaw;
        int readSupportFoot;
        double goalPoseX;
        double goalPoseY;
        double goalPoseYaw;
        int goalSupportFoot;
        double walkOrderX;
        double walkOrderY;
        double walkOrderTheta;
        double walkOrderEnabled;
        double walkPhase;
        double targetX;
        double targetY;
        file >> seq;
        file >> index;
        file >> readPoseX;
        file >> readPoseY;
        file >> readPoseYaw;
        file >> readSupportFoot;
        file >> goalPoseX;
        file >> goalPoseY;
        file >> goalPoseYaw;
        file >> goalSupportFoot;
        file >> walkOrderX;
        file >> walkOrderY;
        file >> walkOrderTheta;
        file >> walkOrderEnabled;
        file >> walkPhase;
        file >> targetX;
        file >> targetY;
        if (lastSeq != seq) {
            readTrajsPose.push_back(std::vector<Eigen::Vector3d>());
            readTrajsSupport.push_back(std::vector<Leph::HumanoidFixedModel::SupportFoot>());
            goalTrajsPose.push_back(std::vector<Eigen::Vector3d>());
            goalTrajsSupport.push_back(std::vector<Leph::HumanoidFixedModel::SupportFoot>());
            walkTrajsOrder.push_back(std::vector<Eigen::Vector4d>());
            walkTrajsPhase.push_back(std::vector<double>());
            targetDisplacements.push_back(Eigen::Vector2d(targetX, targetY));
        }
        lastSeq = seq;
        readTrajsPose.back().push_back(Eigen::Vector3d(readPoseX, readPoseY, readPoseYaw));
        readTrajsSupport.back().push_back((Leph::HumanoidFixedModel::SupportFoot)readSupportFoot);
        goalTrajsPose.back().push_back(Eigen::Vector3d(goalPoseX, goalPoseY, goalPoseYaw));
        goalTrajsSupport.back().push_back((Leph::HumanoidFixedModel::SupportFoot)goalSupportFoot);
        walkTrajsOrder.back().push_back(Eigen::Vector4d(walkOrderX, walkOrderY, walkOrderTheta, walkOrderEnabled));
        walkTrajsPhase.back().push_back(walkPhase);
    }
    file.close();
    std::cout << "Loaded " << readTrajsPose.size() << " sequences" << std::endl;
    for (size_t i=0;i<readTrajsPose.size();i++) {
        std::cout << "Seq " << i << " with " << readTrajsPose[i].size() << " points " 
            << "Displacement: " << targetDisplacements[i].transpose() << std::endl;
    }

    //Fitness function
    bool verbose  = false;
    libcmaes::FitFuncEigen fitness = 
        [&type, 
        &readTrajsPose, &readTrajsSupport, 
        &goalTrajsPose, &goalTrajsSupport, 
        &walkTrajsOrder, &walkTrajsPhase, 
        &targetDisplacements, &verbose]
        (const Eigen::VectorXd& params) 
        {
            //Check bounds
            if (params.lpNorm<Eigen::Infinity>() > 10.0) {
                return 1000.0 + 1000.0*params.lpNorm<Eigen::Infinity>();
            }
            Leph::Plot plot;
            double error = 0.0;
            int count = 0;
            for (size_t i=0;i<readTrajsPose.size();i++) {
                Leph::OdometryModel odometry(type);
                odometry.parameters() = params;
                odometry.reset();
                double lastPhase = walkTrajsPhase[i].front();
                for (size_t j=0;j<readTrajsPose[i].size();j++) {
                    /*
                    double phase = walkTrajsPhase[i][j];
                    if (lastPhase > 0.8 && phase < 0.2) {
                        odometry.updateFullStep(
                            walkTrajsOrder[i][j].segment(0, 3) * walkTrajsOrder[i][j](3));
                    }
                    lastPhase = phase;
                    */
                    odometry.update(
                        readTrajsPose[i][j], 
                        readTrajsSupport[i][j]);
                    /*
                    odometry.update(
                        goalTrajsPose[i][j], 
                        goalTrajsSupport[i][j]);
                    */
                    if (verbose) {
                        plot.add(Leph::VectorLabel(
                            "odometry_x", odometry.state().x(),
                            "odometry_y", odometry.state().y(),
                            "read_x", readTrajsPose[i][j].x(),
                            "read_y", readTrajsPose[i][j].y(),
                            "goal_x", goalTrajsPose[i][j].x(),
                            "goal_y", goalTrajsPose[i][j].y(),
                            "target_x", targetDisplacements[i].x(),
                            "target_y", targetDisplacements[i].y(),
                            "seq", i
                        ));
                    }
                }
                if (verbose) {
                    std::cout << "Seq " << i 
                        << ": error=" << (targetDisplacements[i] - odometry.state().segment(0, 2)).norm()
                        << " target=" << targetDisplacements[i].transpose()
                        << " odometry=" << odometry.state().segment(0, 2).transpose()
                        << std::endl;
                }
                error += (
                    targetDisplacements[i] 
                    - odometry.state().segment(0, 2)
                ).squaredNorm();
                count++;
            }
            if (verbose) {
                plot
                    .plot("odometry_x", "odometry_y", Leph::Plot::LinesPoints, "seq")
                    //.plot("read_x", "read_y", Leph::Plot::LinesPoints, "seq")
                    //.plot("goal_x", "goal_y", Leph::Plot::LinesPoints, "seq")
                    .plot("target_x", "target_y")
                    .render();
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
    verbose = true;
    fitness(bestParams);

    return 0;
}

