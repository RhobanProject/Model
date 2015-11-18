#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "LegIK/LegIK.hpp"
#include "Model/Model.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/InverseKinematics.hpp"
#include "Spline/SmoothSpline.hpp"
#include "Spline/FittedSpline.hpp"
#include "Plot/Plot.hpp"

/**
 * Load given filename in ReM1 format
 */
static std::vector<Leph::Spline> loadSplines(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Unable to open file: " << filename << std::endl;
        exit(-1);
    }

    std::vector<Leph::Spline> container;
    std::string line;
    double t = 0.0;
    while (file.good()) {
        std::getline(file, line);
        if (line.size() == 0) {
            break;
        }
        if (line[0] >= 65 && line[0] <= 122) {
            std::cout << "New Spline: " << line << std::endl;
            container.push_back(Leph::Spline());
            t = 0.0;
        } else {
            double time = std::stod(line); 
            time /= 10000.0;
            std::vector<double> coefs;
            std::string lineCoefs;
            while (true) {
                double val;
                file >> val;
                coefs.push_back(val);
                if (file.peek() == '\n' || file.peek() == -1 || file.peek() == '\r') {
                    file.ignore();
                    break;
                }
                file.ignore();
            }
            Leph::Polynom poly(coefs.size()-1);
            for (size_t i=0;i<coefs.size();i++) {
                poly(i) = coefs[i];
            }
            container.back().addPart(poly, t, t+time);
            t += time;
        }
    }

    return container;
}

/**
 * Run analytical inverse kinematics on given
 * model with given position and orientation (Yaw-Pitch-Roll)
 * targets
 */
static bool inverseKinematics(Leph::Model& model, 
    const Eigen::Vector3d& targetPos, const Eigen::Vector3d& targetAngle)
{
    double legHipToKnee = 0.09375;
    double legKneeToAnkle = 0.105;
    double legAnkleToGround = 0.0319955;
    Eigen::Vector3d footToPen(0.08, 0.037, -0.0085);

    Eigen::Quaternion<double> quat;
    Eigen::AngleAxisd yawRot(targetAngle(0), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchRot(targetAngle(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollRot(targetAngle(2), Eigen::Vector3d::UnitX());
    quat = rollRot * pitchRot * yawRot;
    Eigen::Matrix3d rotation = quat.matrix();

    Eigen::Vector3d footToPenWorld = rotation.transpose()*footToPen;
    Eigen::Vector3d footTargetPos = targetPos - footToPenWorld;

    //LegIK initialization
    LegIK::IK ik(legHipToKnee, 
        legKneeToAnkle, legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = footTargetPos(0);
    legIKTarget[1] = footTargetPos(1);
    legIKTarget[2] = footTargetPos(2);
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix;
    legIKMatrix[0][0] = rotation(0, 0);
    legIKMatrix[0][1] = rotation(0, 1);
    legIKMatrix[0][2] = rotation(0, 2);
    legIKMatrix[1][0] = rotation(1, 0);
    legIKMatrix[1][1] = rotation(1, 1);
    legIKMatrix[1][2] = rotation(1, 2);
    legIKMatrix[2][0] = rotation(2, 0);
    legIKMatrix[2][1] = rotation(2, 1);
    legIKMatrix[2][2] = rotation(2, 2);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result, true);

    //Update degrees of freedom on success
    if (isSucess) {
        model.setDOF("left_hip_yaw", result.theta[0]);
        model.setDOF("left_hip_roll", result.theta[1]);
        model.setDOF("left_hip_pitch", -result.theta[2]);
        model.setDOF("left_knee", result.theta[3]);
        model.setDOF("left_ankle_pitch", -result.theta[4]);
        model.setDOF("left_ankle_roll", result.theta[5]);
    } 

    return isSucess;
}

int main(int argc, char** argv)
{
    std::string mode;
    std::string filename;
    if (argc != 2) {
        std::cout << "Usage: ./app filename" << std::endl;
        return 1;
    }
    filename = std::string(argv[1]);

    std::cout << "Loading " << filename << std::endl;
    std::vector<Leph::Spline> splinesCart = loadSplines(filename);
    double maxTime = splinesCart.front().max();

    //Load raw model
    Leph::Model model("../Data/legWithPen.urdf");
    
    //Spline container initialization
    std::vector<Leph::FittedSpline> splinesPos(6);
    std::vector<Leph::FittedSpline> splinesTorque(6);

    //Viewer
    Leph::ModelViewer viewer(1200, 900);
    //Main Loop
    Leph::Plot plotTorque;
    Leph::Plot plotPos;
    double t = 0.0;
    while (viewer.update() && t < maxTime) {
        //Update target
        double x = splinesCart[0].pos(t);
        double y = splinesCart[1].pos(t);
        double z = splinesCart[2].pos(t);
        
        double minDist = 0.13;
        double maxDist = 0.27;
        double angleAtMin = M_PI/2.0;
        double angleAtMax = M_PI/2.0 - 0.7;
        
        double distTarget = sqrt(x*x + y*y);
        double pitch;
        if (distTarget < minDist) {
            pitch = angleAtMin;
        } else if (distTarget > maxDist) {
            pitch = angleAtMax;
        } else {
            double ratio = (distTarget-minDist)/(maxDist-minDist);
            pitch = (1.0-ratio)*angleAtMin + ratio*angleAtMax;
        }
        double yaw = -atan2(y, x);
        
        bool isSucess = inverseKinematics(model, 
            Eigen::Vector3d(x, y, z), Eigen::Vector3d(yaw,  pitch, 0.0));
        if (!isSucess) {
            std::cout << "IK ERROR at t=" << t << std::endl;
        } 

        //Foot velocity and acceleration vector
        double vx = splinesCart[0].vel(t);
        double vy = splinesCart[1].vel(t);
        double vz = splinesCart[2].vel(t);
        double ax = splinesCart[0].acc(t);
        double ay = splinesCart[1].acc(t);
        double az = splinesCart[2].acc(t);
        Eigen::VectorXd pos = model.getDOFVect();
        Eigen::VectorXd vel(6, 1);
        Eigen::VectorXd acc(6, 1);
        //Roll-Pitch-Yaw convention
        //TODO velocity of orientation pitch and yaw are not considered
        vel(0) = 0.0;
        vel(1) = 0.0;
        vel(2) = 0.0;
        vel(3) = vx;
        vel(4) = vy;
        vel(5) = vz;
        acc(0) = 0.0;
        acc(1) = 0.0;
        acc(2) = 0.0;
        acc(3) = ax;
        acc(4) = ay;
        acc(5) = az;
        //Compute foot jacobian matrix
        Eigen::MatrixXd jac = model.pointJacobian("left_foot_tip");
        //Compute joint velocities
        Eigen::VectorXd dq = jac.fullPivLu().solve(vel);
        //Compute joint acceleration
        //acc = J(q)*ddq + dJ(q, dq)*dq
        //=> ddq = J(q)^-1*(acc - dJ*dq)
        //dJ*dq can be computed using pointAcceleration and setting 
        //ddq to zero (thanks Martin Felis !).
        Eigen::VectorXd J_dot_q_dot = model.pointAcceleration("left_foot_tip", dq, 
            Eigen::VectorXd::Zero(model.sizeDOF()));
        Eigen::VectorXd ddq = jac.fullPivLu().solve(acc - J_dot_q_dot);
        //Compute joint torques
        model.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
        Eigen::VectorXd torques = model.inverseDynamics(dq, ddq);
        //Assign to splines
        splinesTorque[0].addPoint(t, torques(0));
        splinesTorque[1].addPoint(t, torques(1));
        splinesTorque[2].addPoint(t, torques(2));
        splinesTorque[3].addPoint(t, torques(3));
        splinesTorque[4].addPoint(t, torques(4));
        splinesTorque[5].addPoint(t, torques(5));
        splinesPos[0].addPoint(t, pos(0));
        splinesPos[1].addPoint(t, pos(1));
        splinesPos[2].addPoint(t, pos(2));
        splinesPos[3].addPoint(t, pos(3));
        splinesPos[4].addPoint(t, pos(4));
        splinesPos[5].addPoint(t, pos(5));
        //Display model
        Eigen::Vector3d pt = model.position("left_foot_tip", "origin");
        viewer.addTrackedPoint(pt);    
        viewer.maxTrajectory = 10000;
        Leph::ModelDraw(model, viewer);
        t += 0.01;
        //Plot
        plotTorque.add(Leph::VectorLabel(
            "t", t,
            "t0", torques(0),
            "t1", torques(1),
            "t2", torques(2),
            "t3", torques(3),
            "t4", torques(4),
            "t5", torques(5)
        ));
        plotPos.add(Leph::VectorLabel(
            "t", t,
            "q0", pos(0),
            "q1", pos(1),
            "q2", pos(2),
            "q3", pos(3),
            "q4", pos(4),
            "q5", pos(5)
        ));
    }
    
    //Splines fitting
    for (size_t i=0;i<splinesTorque.size();i++) {
        splinesTorque[i].fittingGlobal(4, 50);
        splinesPos[i].fittingGlobal(4, 50);
    }
    
    //Ploting position and torque trajectories
    for (double t=0;t<maxTime;t+=maxTime/1000.0) {
        plotTorque.add(Leph::VectorLabel(
            "t", t, 
            "t0 fitted", splinesTorque[0].pos(t),
            "t1 fitted", splinesTorque[1].pos(t),
            "t2 fitted", splinesTorque[2].pos(t),
            "t3 fitted", splinesTorque[3].pos(t),
            "t4 fitted", splinesTorque[4].pos(t),
            "t5 fitted", splinesTorque[5].pos(t)
        ));
        plotPos.add(Leph::VectorLabel(
            "t", t, 
            "q0 fitted", splinesPos[0].pos(t),
            "q1 fitted", splinesPos[1].pos(t),
            "q2 fitted", splinesPos[2].pos(t),
            "q3 fitted", splinesPos[3].pos(t),
            "q4 fitted", splinesPos[4].pos(t),
            "q5 fitted", splinesPos[5].pos(t)
        ));
    }
    plotPos.plot("t", "all").render();
    plotTorque.plot("t", "all").render();
    
    //Dumping splines
    std::cout << "==== Torque Trajectories ====" << std::endl;
    for (size_t i=0;i<splinesTorque.size();i++) {
        std::cout << "motor_" << i << std::endl;
        for (size_t j=0;j<splinesTorque[i].size();j++) {
            std::cout << splinesTorque[i].part(j).max-splinesTorque[i].part(j).min << std::endl;
            for (size_t k=0;k<splinesTorque[i].part(j).polynom.degree()+1;k++) {
                std::cout << splinesTorque[i].part(j).polynom(k);
                if (k != splinesTorque[i].part(j).polynom.degree()) {
                    std::cout << ", ";
                }
            }
            std::cout << std::endl;
        }
    }
    std::cout << "==== Position Trajectories ====" << std::endl;
    for (size_t i=0;i<splinesPos.size();i++) {
        std::cout << "motor_" << i << std::endl;
        for (size_t j=0;j<splinesPos[i].size();j++) {
            std::cout << splinesPos[i].part(j).max-splinesPos[i].part(j).min << std::endl;
            for (size_t k=0;k<splinesPos[i].part(j).polynom.degree()+1;k++) {
                std::cout << splinesPos[i].part(j).polynom(k);
                if (k != splinesPos[i].part(j).polynom.degree()) {
                    std::cout << ", ";
                }
            }
            std::cout << std::endl;
        }
    }

    return 0;
}

