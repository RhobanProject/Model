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

  // CSV Header
  std::cout << "t,leftAnkle,leftHip,leftLength,rightAnkle,rightHip,rightLength" << std::endl;

  while (viewer.update()) {

    if (t > 20) { break;}

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

    double leftLength = model.position("left_ankle_roll",
                                       "left_hip_roll").norm();
    double rightLength = model.position("right_ankle_roll",
                                        "right_hip_roll").norm();

    std::cout << t << ","
              << model.getDOF("left_ankle_roll")  * 180 / M_PI << ","
              << model.getDOF("left_hip_roll")    * 180 / M_PI << ","
              << leftLength                                    << ","
              << model.getDOF("right_ankle_roll") * 180 / M_PI << ","
              << model.getDOF("right_hip_roll")   * 180 / M_PI << ","
              << rightLength                                   << std::endl;


    double dt = 1.0 / freq * speed;

    toddling.update(dt);

    t += dt;
  }
    
  return 0;
}
