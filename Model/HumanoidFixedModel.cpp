#include "Model/HumanoidFixedModel.hpp"

namespace Leph {

HumanoidFixedModel::HumanoidFixedModel(
    RobotType type) :
    _supportFoot(LeftSupportFoot),
    _modelLeft(type, "left_foot_tip"),
    _modelRight(type, "right_foot_tip")
{
}
        
HumanoidFixedModel::~HumanoidFixedModel()
{
}
        
HumanoidFixedModel::SupportFoot HumanoidFixedModel::
    getSupportFoot() const
{
    return _supportFoot;
}
        
void HumanoidFixedModel::setSupportFoot(SupportFoot foot)
{
    if (_supportFoot != foot) {
        if (foot == RightSupportFoot) {
            Eigen::Vector3d posFoot = 
                _modelLeft.position("right_foot_tip", "origin");
            _modelRight.importDOF(_modelLeft);
            _modelRight.setDOF("base_x", posFoot.x());
            _modelRight.setDOF("base_y", posFoot.y());
            setYaw(foot);
            _supportFoot = RightSupportFoot;
        } else {
            Eigen::Vector3d posFoot = 
                _modelRight.position("left_foot_tip", "origin");
            _modelLeft.importDOF(_modelRight);
            _modelLeft.setDOF("base_x", posFoot.x());
            _modelLeft.setDOF("base_y", posFoot.y());
            setYaw(foot);
            _supportFoot = LeftSupportFoot;
        }
    }
}

void HumanoidFixedModel::setYaw(SupportFoot foot)
{
    if (foot == RightSupportFoot) {
        _modelRight.setDOF("base_yaw", 
            _modelLeft.orientationYaw("right_foot_tip", "origin"));
    } else {
        _modelLeft.setDOF("base_yaw", 
            _modelRight.orientationYaw("left_foot_tip", "origin"));
    }
}
void HumanoidFixedModel::setYaw(SupportFoot foot, double trunkYaw)
{
    if (foot == RightSupportFoot) {
        double yaw = trunkYaw;
        yaw += _modelRight.getDOF("right_hip_yaw");
        _modelRight.setDOF("base_yaw", yaw);
    } else {
        double yaw = trunkYaw;
        yaw += _modelLeft.getDOF("left_hip_yaw");
        _modelLeft.setDOF("base_yaw", yaw);
    }
}
        
const HumanoidModel& HumanoidFixedModel::get() const
{
    if (_supportFoot == LeftSupportFoot) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}
HumanoidModel& HumanoidFixedModel::get()
{
    if (_supportFoot == LeftSupportFoot) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}
        
void HumanoidFixedModel::updateBase()
{
    //Check if moving foot is touching
    //and switch current model with 
    //new supporting foot
    if (_supportFoot == LeftSupportFoot) {
        Eigen::Vector3d posFoot = 
            _modelLeft.position("right_foot_tip", "origin");
        if (posFoot.z() < 0.0) {
            setSupportFoot(RightSupportFoot);
        }
    } else {
        Eigen::Vector3d posFoot = 
            _modelRight.position("left_foot_tip", "origin");
        if (posFoot.z() < 0.0) {
            setSupportFoot(LeftSupportFoot);
        }
    }
}
        
void HumanoidFixedModel::setOrientation(
    double trunkPitch, double trunkRoll)
{
    //Trunk euler angle orientation to 
    //rotation matrix
    Eigen::AngleAxisd pitchRot(-trunkPitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollRot(-trunkRoll, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> quat = rollRot*pitchRot;
    Eigen::Matrix3d baseToTrunk = quat.matrix();
    
    //Computing rotation matrix from support foot tip to trunk
    Eigen::Matrix3d footToTrunk;
    if (_supportFoot == LeftSupportFoot) {
        footToTrunk = _modelLeft.orientation("left_foot_tip", "trunk");
    } else {
        footToTrunk = _modelRight.orientation("right_foot_tip", "trunk");
    }
    
    //Computing rotation matrix to apply on floating base
    //from base to foot
    Eigen::Matrix3d baseToFoot = footToTrunk * baseToTrunk;
    
    //Retrieve euler angles from rotation matrix
    //(Manual computing without singular check seems better than
    //Eigen euler angles and with better range)
    Eigen::Vector3d angles;
    angles(0) = atan2(baseToFoot(1, 2), baseToFoot(2, 2));
    angles(1) = atan2(-baseToFoot(0, 2), 
        sqrt(baseToFoot(0,0)*baseToFoot(0,0) + baseToFoot(0,1)*baseToFoot(0,1)));
    angles(2) = atan2(baseToFoot(0, 1), baseToFoot(0, 0));

    //Assign floating base DOFs with fixed yaw
    if (_supportFoot == LeftSupportFoot) {
        _modelLeft.setDOF("base_roll", angles(0));
        _modelLeft.setDOF("base_pitch", angles(1));
    } else {
        _modelRight.setDOF("base_roll", angles(0));
        _modelRight.setDOF("base_pitch", angles(1));
    }
}

Eigen::Vector3d HumanoidFixedModel::zeroMomentPoint(
    const std::string& frame,
    const Eigen::VectorXd& velocity,
    const Eigen::VectorXd& acceleration)
{
    Eigen::VectorXd torque = get().inverseDynamics(
        velocity, acceleration);

    return zeroMomentPointFromTorques(frame, torque);
}
Eigen::Vector3d HumanoidFixedModel::zeroMomentPoint(
    const std::string& frame,
    const VectorLabel& velocity,
    const VectorLabel& acceleration)
{
    VectorLabel torque = get().inverseDynamics(
        velocity, acceleration);

    double Fx = torque("base_x");
    double Fy = torque("base_y");
    double Fz = torque("base_z");
    double MRoll = torque("base_roll");
    double MPitch = torque("base_pitch");
    
    double Mx;
    double My;
    convertFootMoment(MPitch, MRoll, Mx, My);
    Eigen::Vector3d zmp = computeZMP(Fx, Fy, Fz, Mx, My);
    
    return get().position("origin", frame, zmp);
}
        
Eigen::Vector3d HumanoidFixedModel::zeroMomentPointFromTorques(
    const std::string& frame,
    const Eigen::VectorXd& torques)
{
    double Fx = torques(get().getDOFIndex("base_x"));
    double Fy = torques(get().getDOFIndex("base_y"));
    double Fz = torques(get().getDOFIndex("base_z"));
    double MRoll = torques(get().getDOFIndex("base_roll"));
    double MPitch = torques(get().getDOFIndex("base_pitch"));
    
    double Mx;
    double My;
    convertFootMoment(MPitch, MRoll, Mx, My);
    Eigen::Vector3d zmp = computeZMP(Fx, Fy, Fz, Mx, My);

    return get().position("origin", frame, zmp);

}

bool HumanoidFixedModel::trunkFootIK(
    SupportFoot support,
    const Eigen::Vector3d& trunkPos, 
    const Eigen::Matrix3d& trunkRotation,
    const Eigen::Vector3d& flyingFootPos,
    const Eigen::Matrix3d& flyingFootRotation)
{
    //Set the new support foot flat on the ground
    setSupportFoot(support);
    get().setDOF("base_pitch", 0.0);
    get().setDOF("base_roll", 0.0);
    //Compute the rotation matrix from support
    //foot to trunk transposed
    Eigen::Matrix3d rotationT = trunkRotation.transpose();
    
    //Compute left and right leg inverse kinematics
    //for both support foot.
    bool isSuccessLeft = true;
    bool isSuccessRight = true;
    if (support == LeftSupportFoot) {
        isSuccessLeft = get().legIkLeft(
            "trunk",
            rotationT*(-trunkPos), 
            trunkRotation);
        isSuccessRight = get().legIkRight(
            "left_foot_tip",
            flyingFootPos,
            flyingFootRotation.transpose());
    }
    if (support == RightSupportFoot) {
        isSuccessRight = get().legIkRight(
            "trunk",
            rotationT*(-trunkPos), 
            trunkRotation);
        isSuccessLeft = get().legIkLeft(
            "right_foot_tip",
            flyingFootPos,
            flyingFootRotation.transpose());
    }

    return isSuccessLeft && isSuccessRight;
}

Eigen::VectorXd HumanoidFixedModel::trunkFootIKVel(
    const Eigen::Vector3d& trunkPosVel, 
    const Eigen::Vector3d& trunkAxisAnglesVel,
    const Eigen::Vector3d& flyingFootPosVel,
    const Eigen::Vector3d& flyingFootAxisAnglesVel)
{
    //Support foot selection
    std::string supportName;
    std::string flyingName;
    std::string supportPrefix;
    std::string flyingPrefix;
    if (getSupportFoot() == LeftSupportFoot) {
        supportName = "left_foot_tip";
        flyingName = "right_foot_tip";
        supportPrefix = "left_";
        flyingPrefix = "right_";
    } else if (getSupportFoot() == RightSupportFoot) {
        supportName = "right_foot_tip";
        flyingName = "left_foot_tip";
        supportPrefix = "right_";
        flyingPrefix = "left_";
    }

    //Compute trunk and flying foot 
    //jacobian in support foot frame
    Eigen::MatrixXd jacTrunk = get()
        .pointJacobian("trunk", supportName);
    Eigen::MatrixXd jacFoot = get()
        .pointJacobian(flyingName, supportName);

    //Retrieve DOF internal index
    size_t indexSupportAnkleRoll = get()
        .getDOFIndex(supportPrefix + "ankle_roll");
    size_t indexSupportAnklePitch = get()
        .getDOFIndex(supportPrefix + "ankle_pitch");
    size_t indexSupportKnee = get()
        .getDOFIndex(supportPrefix + "knee");
    size_t indexSupportHipPitch = get()
        .getDOFIndex(supportPrefix + "hip_pitch");
    size_t indexSupportHipRoll = get()
        .getDOFIndex(supportPrefix + "hip_roll");
    size_t indexSupportHipYaw = get()
        .getDOFIndex(supportPrefix + "hip_yaw");
    size_t indexFlyingAnkleRoll = get()
        .getDOFIndex(flyingPrefix + "ankle_roll");
    size_t indexFlyingAnklePitch = get()
        .getDOFIndex(flyingPrefix + "ankle_pitch");
    size_t indexFlyingKnee = get()
        .getDOFIndex(flyingPrefix + "knee");
    size_t indexFlyingHipPitch = get()
        .getDOFIndex(flyingPrefix + "hip_pitch");
    size_t indexFlyingHipRoll = get()
        .getDOFIndex(flyingPrefix + "hip_roll");
    size_t indexFlyingHipYaw = get()
        .getDOFIndex(flyingPrefix + "hip_yaw");

    //Restrict both jacobian to support 
    //and flying foot degrees of freedom
    Eigen::MatrixXd subJacTrunk(6, 6);
    subJacTrunk.col(0) = jacTrunk.col(indexSupportAnkleRoll);
    subJacTrunk.col(1) = jacTrunk.col(indexSupportAnklePitch);
    subJacTrunk.col(2) = jacTrunk.col(indexSupportKnee);
    subJacTrunk.col(3) = jacTrunk.col(indexSupportHipPitch);
    subJacTrunk.col(4) = jacTrunk.col(indexSupportHipRoll);
    subJacTrunk.col(5) = jacTrunk.col(indexSupportHipYaw);
    Eigen::MatrixXd subJacFoot(6, 6);
    subJacFoot.col(0) = jacFoot.col(indexFlyingHipYaw);
    subJacFoot.col(1) = jacFoot.col(indexFlyingHipRoll);
    subJacFoot.col(2) = jacFoot.col(indexFlyingHipPitch);
    subJacFoot.col(3) = jacFoot.col(indexFlyingKnee);
    subJacFoot.col(4) = jacFoot.col(indexFlyingAnklePitch);
    subJacFoot.col(5) = jacFoot.col(indexFlyingAnkleRoll);
    //Check for near singular jacobian
    if (
        fabs(subJacTrunk.fullPivLu().determinant()) < 1e-10 ||
        fabs(subJacFoot.fullPivLu().determinant()) < 1e-10
    ) {
        //Return null velocity (no other good choice ?)
        return Eigen::VectorXd::Zero(get().sizeDOF());
    }

    //Build spatial vector of 
    //the trunk velocity in support foot frame
    Eigen::VectorXd trunkVel(6);
    trunkVel.segment(0, 3) = trunkAxisAnglesVel;
    trunkVel.segment(3, 3) = trunkPosVel;

    //Build spatial vector of the relative velocity of
    //flying foot with respect the the trunk in support foot frame 
    Eigen::VectorXd footVel(6);
    footVel.segment(0, 3) = flyingFootAxisAnglesVel - trunkAxisAnglesVel;
    //Compute relative linear translation velocity
    //of the trunk in support frame.
    //T (trunk), F (foot), O (support)
    //vel(F/T) = vel(F/O) - vel(T/O) - w(O/T) cross TF
    Eigen::Vector3d relPos = 
        get().position(flyingName, supportName)
        - get().position("trunk", supportName);
    footVel.segment(3, 3) = 
        flyingFootPosVel 
        - trunkPosVel 
        - trunkAxisAnglesVel.cross(relPos);

    //Compute support leg joint velocities
    Eigen::VectorXd dqSupport = subJacTrunk.fullPivLu().solve(trunkVel);
    //Compute flying leg joint velocities
    Eigen::VectorXd dqFlying = subJacFoot.fullPivLu().solve(footVel);

    //Assign and return computed velocities
    Eigen::VectorXd dofVel = Eigen::VectorXd::Zero(get().sizeDOF());
    dofVel(indexSupportAnkleRoll) = dqSupport(0);
    dofVel(indexSupportAnklePitch) = dqSupport(1);
    dofVel(indexSupportKnee) = dqSupport(2);
    dofVel(indexSupportHipPitch) = dqSupport(3);
    dofVel(indexSupportHipRoll) = dqSupport(4);
    dofVel(indexSupportHipYaw) = dqSupport(5);
    dofVel(indexFlyingHipYaw) = dqFlying(0);
    dofVel(indexFlyingHipRoll) = dqFlying(1);
    dofVel(indexFlyingHipPitch) = dqFlying(2);
    dofVel(indexFlyingKnee) = dqFlying(3);
    dofVel(indexFlyingAnklePitch) = dqFlying(4);
    dofVel(indexFlyingAnkleRoll) = dqFlying(5);
    return dofVel;
}
        
Eigen::VectorXd HumanoidFixedModel::trunkFootIKAcc(
    const Eigen::VectorXd& dq,
    const Eigen::Vector3d& trunkPosVel, 
    const Eigen::Vector3d& trunkAxisAnglesVel,
    const Eigen::Vector3d& flyingFootPosVel,
    const Eigen::Vector3d& flyingFootAxisAnglesVel,
    const Eigen::Vector3d& trunkPosAcc, 
    const Eigen::Vector3d& trunkAxisAnglesAcc,
    const Eigen::Vector3d& flyingFootPosAcc,
    const Eigen::Vector3d& flyingFootAxisAnglesAcc)
{
    //Support foot selection
    std::string supportName;
    std::string flyingName;
    std::string supportPrefix;
    std::string flyingPrefix;
    if (getSupportFoot() == LeftSupportFoot) {
        supportName = "left_foot_tip";
        flyingName = "right_foot_tip";
        supportPrefix = "left_";
        flyingPrefix = "right_";
    } else if (getSupportFoot() == RightSupportFoot) {
        supportName = "right_foot_tip";
        flyingName = "left_foot_tip";
        supportPrefix = "right_";
        flyingPrefix = "left_";
    }

    //Compute trunk and flying foot 
    //jacobian in support foot frame
    Eigen::MatrixXd jacTrunk = get()
        .pointJacobian("trunk", supportName);
    Eigen::MatrixXd jacFoot = get()
        .pointJacobian(flyingName, supportName);

    //Retrieve DOF internal index
    size_t indexSupportAnkleRoll = get()
        .getDOFIndex(supportPrefix + "ankle_roll");
    size_t indexSupportAnklePitch = get()
        .getDOFIndex(supportPrefix + "ankle_pitch");
    size_t indexSupportKnee = get()
        .getDOFIndex(supportPrefix + "knee");
    size_t indexSupportHipPitch = get()
        .getDOFIndex(supportPrefix + "hip_pitch");
    size_t indexSupportHipRoll = get()
        .getDOFIndex(supportPrefix + "hip_roll");
    size_t indexSupportHipYaw = get()
        .getDOFIndex(supportPrefix + "hip_yaw");
    size_t indexFlyingAnkleRoll = get()
        .getDOFIndex(flyingPrefix + "ankle_roll");
    size_t indexFlyingAnklePitch = get()
        .getDOFIndex(flyingPrefix + "ankle_pitch");
    size_t indexFlyingKnee = get()
        .getDOFIndex(flyingPrefix + "knee");
    size_t indexFlyingHipPitch = get()
        .getDOFIndex(flyingPrefix + "hip_pitch");
    size_t indexFlyingHipRoll = get()
        .getDOFIndex(flyingPrefix + "hip_roll");
    size_t indexFlyingHipYaw = get()
        .getDOFIndex(flyingPrefix + "hip_yaw");

    //Restrict both jacobian to support 
    //and flying foot degrees of freedom
    Eigen::MatrixXd subJacTrunk(6, 6);
    subJacTrunk.col(0) = jacTrunk.col(indexSupportAnkleRoll);
    subJacTrunk.col(1) = jacTrunk.col(indexSupportAnklePitch);
    subJacTrunk.col(2) = jacTrunk.col(indexSupportKnee);
    subJacTrunk.col(3) = jacTrunk.col(indexSupportHipPitch);
    subJacTrunk.col(4) = jacTrunk.col(indexSupportHipRoll);
    subJacTrunk.col(5) = jacTrunk.col(indexSupportHipYaw);
    Eigen::MatrixXd subJacFoot(6, 6);
    subJacFoot.col(0) = jacFoot.col(indexFlyingHipYaw);
    subJacFoot.col(1) = jacFoot.col(indexFlyingHipRoll);
    subJacFoot.col(2) = jacFoot.col(indexFlyingHipPitch);
    subJacFoot.col(3) = jacFoot.col(indexFlyingKnee);
    subJacFoot.col(4) = jacFoot.col(indexFlyingAnklePitch);
    subJacFoot.col(5) = jacFoot.col(indexFlyingAnkleRoll);
    //Check for near singular jacobian
    if (
        fabs(subJacTrunk.fullPivLu().determinant()) < 1e-10 ||
        fabs(subJacFoot.fullPivLu().determinant()) < 1e-10
    ) {
        //Return null acceleration (no other good choice ?)
        return Eigen::VectorXd::Zero(get().sizeDOF());
    }
    
    //Build spatial vector of the trunk 
    //acceleration in support foot frame
    Eigen::VectorXd trunkAcc(6);
    trunkAcc.segment(0, 3) = trunkAxisAnglesAcc;
    trunkAcc.segment(3, 3) = trunkPosAcc;

    //Build spatial vector of the relative acceleration of
    //flying foot with respect the the trunk in support foot frame 
    Eigen::VectorXd footAcc(6);
    footAcc.segment(0, 3) = flyingFootAxisAnglesAcc - trunkAxisAnglesAcc;
    //Compute relative linear translation acceleration
    //of the trunk in support frame. 
    //T (trunk), F (foot), O (support)
    //acc(F/T) = acc(F/O) - acc(T/O) - dw/dt(T/O) cross TF 
    //-w(T/O) cross w(T/O) cross TF - 2*w(T/O) cross vel(F/T)
    Eigen::Vector3d relPos = 
        get().position(flyingName, supportName)
        - get().position("trunk", supportName);
    Eigen::Vector3d relVel =
        flyingFootPosVel - trunkPosVel 
        - trunkAxisAnglesVel.cross(relPos);
    footAcc.segment(3, 3) = 
        flyingFootPosAcc
        - trunkPosAcc
        - trunkAxisAnglesAcc.cross(relPos)
        - trunkAxisAnglesVel.cross(trunkAxisAnglesVel.cross(relPos))
        - 2.0*trunkAxisAnglesVel.cross(relVel);

    //Build partial joint velocity vectors
    Eigen::VectorXd dqSupport = Eigen::VectorXd::Zero(get().sizeDOF());
    Eigen::VectorXd dqFlying = Eigen::VectorXd::Zero(get().sizeDOF());
    dqSupport(indexSupportAnkleRoll) = dq(indexSupportAnkleRoll);
    dqSupport(indexSupportAnklePitch) = dq(indexSupportAnklePitch);
    dqSupport(indexSupportKnee) = dq(indexSupportKnee);
    dqSupport(indexSupportHipPitch) = dq(indexSupportHipPitch);
    dqSupport(indexSupportHipRoll) = dq(indexSupportHipRoll);
    dqSupport(indexSupportHipYaw) = dq(indexSupportHipYaw);
    dqFlying(indexFlyingAnkleRoll) = dq(indexFlyingAnkleRoll);
    dqFlying(indexFlyingAnklePitch) = dq(indexFlyingAnklePitch);
    dqFlying(indexFlyingKnee) = dq(indexFlyingKnee);
    dqFlying(indexFlyingHipPitch) = dq(indexFlyingHipPitch);
    dqFlying(indexFlyingHipRoll) = dq(indexFlyingHipRoll);
    dqFlying(indexFlyingHipYaw) = dq(indexFlyingHipYaw);

    //Compute joint acceleration
    //acc = J(q)*ddq + dJ(q, dq)*dq
    //=> ddq = J(q)^-1*(acc - dJ*dq)
    //dJ*dq can be computed using pointAcceleration and setting 
    //ddq to zero (thanks Martin Felis !).
    //Compute support leg joint accelerations
    Eigen::VectorXd J_dot_q_dot_Trunk = get().pointAcceleration("trunk", 
        supportName, dqSupport, Eigen::VectorXd::Zero(get().sizeDOF()));
    Eigen::VectorXd ddqSupport = subJacTrunk.fullPivLu()
        .solve(trunkAcc - J_dot_q_dot_Trunk);
    //Compute flying leg joint accelerations
    Eigen::VectorXd J_dot_q_dot_Foot = get().pointAcceleration(flyingName, 
        supportName, dqFlying, Eigen::VectorXd::Zero(get().sizeDOF()));
    Eigen::VectorXd ddqFlying = subJacFoot.fullPivLu()
        .solve(footAcc - J_dot_q_dot_Foot);

    //Assign and return computed accelerations
    Eigen::VectorXd dofAcc = Eigen::VectorXd::Zero(get().sizeDOF());
    dofAcc(indexSupportAnkleRoll) = ddqSupport(0);
    dofAcc(indexSupportAnklePitch) = ddqSupport(1);
    dofAcc(indexSupportKnee) = ddqSupport(2);
    dofAcc(indexSupportHipPitch) = ddqSupport(3);
    dofAcc(indexSupportHipRoll) = ddqSupport(4);
    dofAcc(indexSupportHipYaw) = ddqSupport(5);
    dofAcc(indexFlyingHipYaw) = ddqFlying(0);
    dofAcc(indexFlyingHipRoll) = ddqFlying(1);
    dofAcc(indexFlyingHipPitch) = ddqFlying(2);
    dofAcc(indexFlyingKnee) = ddqFlying(3);
    dofAcc(indexFlyingAnklePitch) = ddqFlying(4);
    dofAcc(indexFlyingAnkleRoll) = ddqFlying(5);
    return dofAcc;
}

void HumanoidFixedModel::convertFootMoment(
    double torquePitch, double torqueRoll,
    double& Mx, double& My)
{
    //Compute matrix rotation from world origin
    //to foot
    Eigen::Matrix3d mat;
    if (_supportFoot == LeftSupportFoot) {
        mat = get().orientation("left_foot_tip", "origin");
    } else {
        mat = get().orientation("right_foot_tip", "origin");
    }
    mat.transposeInPlace();

    //Compute Pitch/Roll torque vector in world frame
    Eigen::Vector3d uRoll = mat.col(0);
    Eigen::Vector3d uPitch = mat.col(1);
    Eigen::Vector3d torque = torqueRoll*uRoll + torquePitch*uPitch;

    //Retrieve X/Y component
    Mx = torque.x();
    My = torque.y();
}
        
Eigen::Vector3d HumanoidFixedModel::computeZMP(
    double Fx, double Fy, double Fz, 
    double Mx, double My)
{
    Eigen::Vector3d posA;
    if (_supportFoot == LeftSupportFoot) {
        posA = get().position("left_foot_tip", "origin");
    } else {
        posA = get().position("right_foot_tip", "origin");
    }

    double Ax = posA.x();
    double Ay = posA.y();
    double Az = posA.z();

    double Px = (-My - (Az*Fx-Ax*Fz))/Fz;
    double Py = (Mx + (Ay*Fz-Az*Fy))/Fz;
    double Pz = 0.0;

    return Eigen::Vector3d(Px, Py, Pz);
}

}

