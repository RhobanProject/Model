#include <iostream>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Model/RBDLRootUpdate.h"
#include "Model/ModelBuilder.hpp"
#include "Model/InverseKinematics.hpp"
#include "Moves/Toddling.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

#include "Utils/Scheduling.hpp"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

int main()
{
  bool floatingBase = true;
  Leph::Model model= Leph::generateGrobanWithToe(floatingBase);
  //std::cout << model.getDOF() << std::endl;
  //std::cout << "Total mass: " << model.sumMass() << std::endl;
    
  //Viewer loop
  Leph::ModelViewer viewer(1200, 900);

  double t = 0;

  double speed = 0.1;
  double freq = 50.0;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(freq);

  Leph::Toddling toddling;

  while (viewer.update()) {

    Leph::InverseKinematics ik(model);
    toddling.initIK(model, ik);

    ik.run(0.00001, 100);

    Leph::ModelDraw(model, viewer);

    Eigen::Vector3d com = model.centerOfMass("origin");
    Eigen::Vector3d projectedCom(com.x(), com.y(), 0);
    Eigen::Vector3d cop = toddling.expectedCOP();

    viewer.addTrackedPoint(projectedCom,
                           Leph::ModelViewer::Yellow);
    viewer.addTrackedPoint(com, 
                           Leph::ModelViewer::Red);
    viewer.addTrackedPoint(cop, 
                           Leph::ModelViewer::Blue);

    //std::cout << "Targets" << std::endl << ik.getNamedTargets() << std::endl;
    //std::cout << "DOFs"    << std::endl << ik.getNamedDOFSubset() << std::endl;
    //std::cout << "Errors"  << std::endl << ik.getNamedErrors() << std::endl;

    double dt = 1.0 / freq * speed;

    toddling.update(dt);

    t += dt;
  }
    
  return 0;
}
