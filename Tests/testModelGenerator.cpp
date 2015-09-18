#include <iostream>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Model/RBDLRootUpdate.h"
#include "Model/Model.hpp"
#include "Model/ModelBuilder.hpp"
#include "Model/HumanoidModelWithToe.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

int main()
{
  bool floatingBase = true;
  RBDL::Model rbdlModel= Leph::generateGrobanWithToe(floatingBase);
  Leph::Model model(rbdlModel);
  std::cout << model.getDOF() << std::endl;
    
  //Viewer loop
  Leph::ModelViewer viewer(1200, 900);

  double t = 0;
  double period = 200;
  double amplitude = 30 * M_PI / 180;

  while (viewer.update()) {

    //model.setDOF("head_yaw", amplitude * sin(1 * M_PI * t / period));
    //model.setDOF("head_pitch", amplitude * sin(2 * M_PI * t / period));
    //model.setDOF("left_hip_pitch", amplitude * sin(4 * M_PI * t / period));
    Leph::ModelDraw(model, viewer);
    t += 1;
  }
    
  return 0;
}
