#include <iostream>
#include <rbdl/rbdl.h>
#include <urdfreader/urdfreader.h>
#include "Model/RBDLRootUpdate.h"
#include "Model/Model.hpp"
#include "Model/ModelBuilder.hpp"
#include "Model/HumanoidModelWithToe.hpp"
#include "Model/InverseKinematics.hpp"
#include "Utils/Chrono.hpp"
#include "Utils/Scheduling.hpp"

#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"

#include "ToeWalk/ToeHeelWalk.hpp"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

int main()
{
  bool floatingBase = true;
  RBDL::Model rbdlModel= Leph::generateGrobanWithToe(floatingBase);
  Leph::Model model(rbdlModel);
  std::cout << model.getDOF() << std::endl;
  std::cout << "Total mass: " << model.sumMass() << std::endl;

  Leph::ToeHeelWalk walk;
    
  //Viewer loop
  Leph::ModelViewer viewer(1200, 900);

  double t = 0;
  //double period = 200;
  //double amplitude = 30 * M_PI / 180;

  double speed = 0.5;

  double freq = 50.0;
  Leph::Scheduling scheduling;
  scheduling.setFrequency(freq);
  bool forbiddenNextPhase = false;//Emulating onKeyDown with keyPressed
  while (viewer.update()) {

    Leph::InverseKinematics ik(model);

    if (viewer.isKeyPressed(sf::Keyboard::N)) {
      if (!forbiddenNextPhase) {
        walk.nextPhase(model, t);
        forbiddenNextPhase = true;
      }
    }
    else {
      forbiddenNextPhase = false;
    }

    walk.initIK(model, ik, t);

    std::cout << "Current phase: " << walk.getPhaseName() << std::endl;
    std::cout << "ERRORS" << std::endl;
    std::cout << ik.getNamedErrors() << std::endl;
    std::cout << "WEIGHTS" << std::endl;
    std::cout << ik.getNamedWeights() << std::endl;

    std::cout << "left_heel_pos: " << model.position("left_heel","origin").transpose() << std::endl;

    //ik.randomDOFNoise();
    ik.run(0.00001, 100);

    Leph::ModelDraw(model, viewer);
    t += 1.0 / freq * speed;
  }
    
  return 0;
}
