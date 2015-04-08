#include "Model/RBDLClosedLoop.h"

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

namespace Leph {

/**
 * Compute and return Featherstone's K acceleration 
 * constraint equation representing given body id fixed
 * in base coordinates (building closed loop system)
 *
 * K * QDDot = k
 *
 * For fixed constraint, k is null
 * No stabilization term for Inverse Dynamics computation
 */
static RBDLMath::MatrixNd RBDLClosedLoopConstraintEquation(
    RBDL::Model& model, 
    unsigned int fixedBodyId,
    const RBDLMath::VectorNd& Q)
{
    //Compute spatial jacobian for fixed body
    //See Featherstone's book "Rigid Body Dynamics Algorithms"
    //Chapter 8.2 page 145
    RBDLMath::MatrixNd G(6, model.qdot_size);
    G.setZero();
    RBDL::CalcBodySpatialJacobian(
        model, Q, fixedBodyId, G, true);

    return G;
}

/**
 * See See Featherstone's book "Rigid Body Dynamics Algorithms"
 * Chapter 8 for formulae and notations
 */
RBDLMath::VectorNd RBDLClosedLoopInverseDynamics(
    RBDL::Model& model,
    const RBDLMath::VectorNd& Q,
    const RBDLMath::VectorNd& QDot,
    const RBDLMath::VectorNd& QDDot,
    unsigned int fixedBodyId)
{
    //Retrieve the total number of DOF
    //and the number of floating base DOF
    unsigned int allDOFCount = model.dof_count;
    unsigned int virtualDOFCount = 0;
    for (unsigned int i=1;i<model.mBodies.size();i++) {
        if (
            model.mBodies[i].mIsVirtual || 
            model.mBodies[i-1].mIsVirtual
        ) {
            virtualDOFCount++;
        }
        if (!model.mBodies[i].mIsVirtual) {
            break;
        }
    }
    unsigned int usedDOFCount = allDOFCount - virtualDOFCount;
    
    //Compute constraint equation
    RBDLMath::MatrixNd tmpK = RBDLClosedLoopConstraintEquation(
        model, fixedBodyId, Q);
    //Trim constraints to remove floating base DOF
    RBDLMath::MatrixNd K = tmpK.rightCols(usedDOFCount);

    //Compute K kernel basis G such as K*G = 0
    Eigen::FullPivLU<RBDLMath::MatrixNd> lu(K);
    RBDLMath::MatrixNd G = lu.kernel();

    //Compute Inverse Dynamics for kinematics tree model
    //without loop and constraint
    RBDLMath::VectorNd tmpTauOld(allDOFCount);
    tmpTauOld.setZero();
    RBDL::InverseDynamics(
        model, Q, QDot, QDDot,
        tmpTauOld, NULL);
    
    //Trim resulting torques to real used DOF as InverseDynamics
    //torques does only depend on chidren bodies
    //(floating base in single support does not change 
    //InverseDynamics result)
    RBDLMath::VectorNd tauOld = tmpTauOld.tail(usedDOFCount);

    //Compute right side of equation to solve
    //G' * tau = G' * tauOld = tauG
    RBDLMath::VectorNd tauG = G.transpose()*tauOld;

    //Build SVD decomposition of G' in order to retrive it
    //pseudo inverse
    Eigen::JacobiSVD<RBDLMath::MatrixNd> svd(G.transpose(), 
        Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Solve the equation G' * tau = tauG with least square 
    //since the equation are underdetermined (infinite solution) 
    //and we are looking for smallest possible torques configuration
    RBDLMath::VectorNd tmpResultTau = svd.solve(tauG);

    //Pad the result with not computed floating base DOF
    RBDLMath::VectorNd resultTau(allDOFCount);
    resultTau.setZero();
    resultTau.tail(usedDOFCount) = tmpResultTau;

    return resultTau;
}

}

