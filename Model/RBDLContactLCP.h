#ifndef LEPH_RBDLCONTACTLCP_H
#define LEPH_RBDLCONTACTLCP_H

#include <rbdl/rbdl.h>
#include <Eigen/Dense>

namespace Leph {

/**
 * Solve the Linear Complimentary 
 * Problem used to compute which contact
 * constraints have to be activated or is going
 * to release.
 * Use Moby-Drake LCP solver.
 * RBDL model, position, velocity and applied
 * torque are given in addition to the constraint
 * set for only considered contact.
 *
 * Do not add in the given constraint set 
 * the lateral (non inequality) contacts.
 *
 * See Featherstone's book Chapter 11.3, for
 * details and notations.
 *
 * The computed contact force lambda (with zero
 * and non zero elements) is assigned in force
 * field of constraint set.
 */
void RBDLContactLCP(
    RigidBodyDynamics::Model& model,
    const RigidBodyDynamics::Math::VectorNd& Q,
    const RigidBodyDynamics::Math::VectorNd& QDot,
    const RigidBodyDynamics::Math::VectorNd& Tau,
    RigidBodyDynamics::ConstraintSet& CS);

}

#endif

