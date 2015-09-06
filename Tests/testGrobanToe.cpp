#include <iostream>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Model/RBDLRootUpdate.h"
#include "Model/Model.hpp"
#include "Model/HumanoidModelWithToe.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

namespace RBDL = RigidBodyDynamics;

int main()
{
//    //URDF loading
//    RBDL::Model model;
//    if (!RBDL::Addons::URDFReadFromFile(
//        "GrosbanToe.urdf", &model, false)
//    ) {
//        std::runtime_error("Model unable to load URDF file");
//    }
//
//    std::cout << RBDL::Utils::GetModelDOFOverview(model) << std::endl;
//    std::cout << RBDL::Utils::GetModelHierarchy(model) << std::endl;
//
    //Load model into wrapping class
    //Leph::Model wrappedModel(model);
  Leph::HumanoidModelWithToe wrappedModel("GrosbanToe.urdf","ROOT");
    std::cout << wrappedModel.getDOF() << std::endl;
    
    //Viewer loop
    Leph::ModelViewer viewer(1200, 900);

    double t = 0;
    double period = 100;
    double amplitude = 40 * M_PI / 180;
    while (viewer.update()) {
      //wrappedModel.setDOF("left_hip_yaw", wrappedModel.getDOF("left_hip_yaw"));
      double angle = (sin(t / period) * amplitude + amplitude) / 2;
      wrappedModel.setDOF("left_hip_pitch", - angle);
      wrappedModel.setDOF("left_knee", 2 * angle);
      wrappedModel.setDOF("left_ankle_pitch", - angle);
      Leph::ModelDraw(wrappedModel, viewer);
      t += 1;
    }
    
    return 0;
}

