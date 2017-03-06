#include <stdexcept>
#include <sstream>
#include "Model/RBDLContactLCP.h"
#include "LCPMobyDrake/LCPSolver.hpp"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

namespace Leph {

/**
 * Compute contraint vector GDot * QDot
 * from given position and velocity.
 * Adapted from RBDL contactJabobian() function 
 * in order to compute GDot elements in same
 * order as G.
 *
 * RBDL model, position, velocity and 
 * constraints set are given.
 */
static void CalcContactJacobianDotQDot(
    RBDL::Model& model,
    const RBDLMath::VectorNd& Q,
    const RBDLMath::VectorNd& QDot,
    const RBDL::ConstraintSet& CS,
    RBDLMath::VectorNd& GDotQDot,
    bool update_kinematics) 
{
    if (update_kinematics) {
        UpdateKinematicsCustom (model, &Q, NULL, NULL);
    }

    unsigned int i;
    GDotQDot.setZero();
    for (i = 0; i < CS.size(); i++) {
        //Compute the JacDot*QDot term with
        //following formula
        //cartesianAcceleration = J * QDDot + JDot * QDot
        //Compute point acceleration with zero acceleration.
        //Here, matrix G = Jac (in base frame)
        //for each constraints.
        RBDLMath::VectorNd zeros = 
            RBDLMath::VectorNd::Zero(model.dof_count);
        RBDLMath::Vector3d tmpAcc = RBDL::CalcPointAcceleration(
            model, Q, QDot, zeros, CS.body[i], CS.point[i], true);
        GDotQDot(i) = tmpAcc.transpose() * CS.normal[i];
    }
}

void RBDLContactLCP(
    RBDL::Model& model,
    const RBDLMath::VectorNd& Q,
    const RBDLMath::VectorNd& QDot,
    const RBDLMath::VectorNd& Tau,
    RBDL::ConstraintSet& CS)
{
    //Size check
    if (
        Q.size() != model.dof_count ||
        QDot.size() != model.dof_count ||
        Tau.size() != model.dof_count
    ) {
        throw std::logic_error(
            "RBDLContactLCP invalid inputs size");
    }

    //Compute H, C, and G matrixes.
    RBDL::CalcContactSystemVariables(
        model, Q, QDot, Tau, CS);

    //Featherstone's book chapter 11.3 and 11.5.
    //The LCP to solve is:
    //zetaDot = M*lambda + d
    //with: zetaDot(i) >=0, lambda(i) >= 0, zetaDot(i)*lambda(i) = 0.
    //lambda: contact force.
    //zetaDot: contact distance acceleration.
    //zeta: contact distance velocity = G*QDot.
    //M = G*Hinv*Gt.
    //d = G*Hinv*(Tau-C) + GDot*QDot.
    //Note that in Featherstone's boot, 
    //notation is G = T.transpose().
    
    //Check that given position and velocity 
    //comply with given constraint.
    //If not, an impuse need to be apply.
    RBDLMath::VectorNd Zeta = CS.G * QDot;
    if (Zeta.lpNorm<Eigen::Infinity>() > 1e-6) {
        std::ostringstream ss;
        ss << " Zeta: " << Zeta.transpose();
        ss << " setSize: " << CS.size();
        throw std::logic_error(
            "RBDLContactLCP velocity not constrainted. Impulse needed." 
            + ss.str());
    }

    size_t sizeCst = CS.size();
    RBDLMath::MatrixNd M(sizeCst, sizeCst);
    RBDLMath::VectorNd d(sizeCst);
    //Compute H inverse
    auto HQR = CS.H.colPivHouseholderQr();
    if (!HQR.isInvertible()) {
        throw std::logic_error(
            "RBDLContactLCP non invertible H matrix");
    }
    RBDLMath::MatrixNd Hinv = HQR.inverse();
    //Compute GDot*QDot vector term
    RBDLMath::VectorNd GDotQDot(sizeCst);
    CalcContactJacobianDotQDot(
        model, Q, QDot, CS, GDotQDot, true);
    //Compute M matrix and d vector
    M = CS.G * Hinv * CS.G.transpose();
    d = CS.G * Hinv * (Tau - CS.C) + GDotQDot;
    
    //Compute LCP solution
    RBDLMath::VectorNd lambda(sizeCst);
    Drake::MobyLCPSolver solver;
    bool isSuccess = solver.
        SolveLcpLemkeRegularized(M, d, &lambda);
    if (!isSuccess) {
        std::ostringstream ss;
        ss << std::endl;
        ss << "size:" << sizeCst << std::endl;
        ss << "success:" << isSuccess << std::endl;
        ss << "lambda:" << lambda.transpose() << std::endl;
        ss << "zetaDot:" << (M*lambda+d).transpose() << std::endl;
        ss << "zeta:" << (CS.G * QDot).transpose() << std::endl;
        throw std::logic_error(
            "RBDLContactLCP LCP not success." 
            + ss.str());
    }

    //Assign computed lambda in
    //constraint set
    for (size_t i=0;i<CS.size();i++) {
        CS.force(i) = lambda(i);
    }
}

}

