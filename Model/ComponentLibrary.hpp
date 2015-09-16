#pragma once

#include <rbdl/rbdl.h>

#include <map>

namespace Leph {

  namespace RBDL = RigidBodyDynamics;
  namespace RBDLMath = RigidBodyDynamics::Math;

  class ComponentLibrary {
  private:
    static std::map<std::string,RBDL::Body> bodies;
    static void createBodies();

  public:

    static RBDL::Joint roll, pitch, yaw, floatingBase;

    static const RBDL::Body getBody(const std::string & name);

  };
}
