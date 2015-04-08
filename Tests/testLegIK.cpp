#include <iostream>
#include "LegIK/LegIK.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/SigmabanFixedModel.hpp"
#include "Utils/Scheduling.hpp"

int main()
{
    //Initialize instance
    Leph::SigmabanFixedModel model;
    Leph::ModelViewer viewer(1200, 900);

    //Compute model fixed length
    model.get().setDOFZeros();
    Eigen::Vector3d hipPt = model.get().position("right hip roll", "origin");
    Eigen::Vector3d kneePt = model.get().position("right knee", "origin");
    Eigen::Vector3d anklePt = model.get().position("right foot pitch", "origin");
    Eigen::Vector3d footPt = model.get().position("right foot tip", "origin");
    double sigmabanL0 = (hipPt-kneePt).norm();
    double sigmabanL1 = (kneePt-anklePt).norm();
    double sigmabanL2 = (anklePt-footPt).norm();
    std::cout << sigmabanL0 << std::endl;
    std::cout << sigmabanL1 << std::endl;
    std::cout << sigmabanL2 << std::endl;
    LegIK::IK ik(sigmabanL0, sigmabanL1, sigmabanL2);
    LegIK::Position result;
    
    double freq = 50.0;
    Leph::Scheduling scheduling;
    scheduling.setFrequency(freq);
    double t = 0.0;
    while (viewer.update()) {
        t += 0.01;
        bool isSucess = ik.compute(
            LegIK::Vector3D(0.1*sin(t), 0.05*sin(2*t), -.1507525), 
            LegIK::Frame3D::from_euler(0.5*sin(t), 0.0, 0.0),
            result);
        if (!isSucess) {
            std::cout << "TestsLegIK IK error" << std::endl;
            return -1;
        }
        model.get().setDOF("left foot pitch", 0.5);
        model.get().setDOF("left hip yaw", 0.5);
        model.get().setDOF("right hip yaw", result.theta[0]);
        model.get().setDOF("right hip roll", result.theta[1]);
        model.get().setDOF("right hip pitch", -result.theta[2]);
        model.get().setDOF("right knee", result.theta[3]);
        model.get().setDOF("right foot pitch", -result.theta[4]);
        model.get().setDOF("right foot roll", result.theta[5]);
        
        Eigen::Vector3d com = model.get().position("right foot tip", "origin");
        viewer.addTrackedPoint(com);    
        //Display model
        Leph::ModelDraw(model.get(), viewer);
        //Waiting
        scheduling.wait();
    }

    return 0;
}

