#include "Viewer/ModelDraw.hpp"

namespace Leph {

void ModelDraw(Model& model, ModelViewer& viewer)
{
    const RBDL::Model& rbdlModel = model.getRBDLModel();
    size_t originIndex = model.getFrameIndex("origin");

    //Draw tracked point trajectory
    viewer.drawTrajectory();

    //Draw all references frames
    for (size_t i=0;i<model.sizeFrame();i++) {
        Eigen::Vector3d pos = model.position(i, originIndex);
        Eigen::Matrix3d mat = model.orientation(i, originIndex);
        viewer.drawFrame(pos, mat);
        
        //Draw optional bounding box
        double sizeX;
        double sizeY;
        double sizeZ;
        Eigen::Vector3d center;
        model.boundingBox(i, sizeX, sizeY, sizeZ, center);
        viewer.drawBox(sizeX, sizeY, sizeZ, pos+center, mat);
    }

    //Draw center of mass
    viewer.drawMass(model.centerOfMass("origin"), 
        RBDLMath::Matrix3d::Identity());

    for (size_t i=0;i<rbdlModel.mBodies.size();i++) {
        //Virtual moby used for multi DOF are skipped
        if (rbdlModel.mBodies[i].mMass < 0.0001) {
            continue;
        }
        //Draw RBDL bodies center of mass
        size_t bodyIndex = model.bodyIdToFrameIndex(i);
        Eigen::Vector3d com = rbdlModel.mBodies[i].mCenterOfMass;
        Eigen::Vector3d pos = model.position(bodyIndex, originIndex, com);
        Eigen::Matrix3d mat = model.orientation(bodyIndex, originIndex);
        viewer.drawMass(pos, mat);
        Eigen::Vector3d center = model.position(bodyIndex, originIndex);
        //Draw RBDL joints if parent body is non virtual
        if (rbdlModel.mBodies[rbdlModel.lambda[i]].mMass > 0.0001) {
            RBDLMath::Vector3d jointAxis = rbdlModel.S[i].head(3);
            RBDLMath::Matrix3d transformAxis = RBDLMath::Matrix3d::Identity();
            if (
                (jointAxis-RBDLMath::Vector3d(1.0, 0.0, 0.0)).norm() > 0.001 &&
                (jointAxis-RBDLMath::Vector3d(-1.0, 0.0, 0.0)).norm() > 0.001
            ) {
                transformAxis = Eigen::AngleAxisd(-M_PI/2.0, 
                        jointAxis.cross(RBDLMath::Vector3d(1.0, 0.0, 0.0)))
                    .toRotationMatrix();
            }
            if (
                (jointAxis-RBDLMath::Vector3d(-1.0, 0.0, 0.0)).norm() < 0.001
            ) {
                transformAxis *= -1.0;
            }
            transformAxis.transposeInPlace();
            transformAxis *= mat;
            viewer.drawJoint(center, transformAxis);
        }
        //Draw link between body origin and center of mass
        viewer.drawLink(center, pos);
        //Draw link between body center of mass and its children origin
        for (size_t j=0;j<rbdlModel.mu[i].size();j++) {
            size_t childIndex = model.bodyIdToFrameIndex(rbdlModel.mu[i][j]);
            Eigen::Vector3d centerChild = model.position(childIndex, originIndex);
            viewer.drawLink(pos, centerChild);
        }
    }

    //Draw link between fixed reference frame and the center of 
    //mass of it movable parent
    for (size_t i=0;i<rbdlModel.mFixedBodies.size();i++) {
        size_t parentId = rbdlModel.mFixedBodies[i].mMovableParent;
        size_t parentIndex = model.bodyIdToFrameIndex(parentId);
        size_t bodyIndex = model.bodyIdToFrameIndex(i + rbdlModel.fixed_body_discriminator);
        Eigen::Vector3d com = rbdlModel.mBodies[parentId].mCenterOfMass;
        Eigen::Vector3d pos = model.position(parentIndex, originIndex, com);
        Eigen::Vector3d center = model.position(bodyIndex, originIndex);
        viewer.drawLink(pos, center);
    }
}

void CameraDraw(
    const CameraParameters& params,
    HumanoidModel& model, 
    ModelViewer& viewer)
{
    Eigen::Vector3d groundPos1;
    Eigen::Vector3d groundPos2;
    Eigen::Vector3d groundPos3;
    Eigen::Vector3d groundPos4;
    Eigen::Vector3d groundPos5;
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        params, Eigen::Vector2d(-1.0, -1.0)),
        groundPos1);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        params, Eigen::Vector2d( 1.0, -1.0)), 
        groundPos2);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        params, Eigen::Vector2d( 1.0,  1.0)), 
        groundPos3);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        params, Eigen::Vector2d(-1.0,  1.0)), 
        groundPos4);
    model.cameraViewVectorToWorld(model.cameraPixelToViewVector(
        params, Eigen::Vector2d( 0.0,  0.0)), 
        groundPos5);
    viewer.drawLink(model.position("camera", "origin"), groundPos1);
    viewer.drawLink(model.position("camera", "origin"), groundPos2);
    viewer.drawLink(model.position("camera", "origin"), groundPos3);
    viewer.drawLink(model.position("camera", "origin"), groundPos4);
    viewer.drawLink(model.position("camera", "origin"), groundPos5);
    viewer.drawLink(groundPos1, groundPos2);
    viewer.drawLink(groundPos2, groundPos3);
    viewer.drawLink(groundPos3, groundPos4);
    viewer.drawLink(groundPos4, groundPos1);
}

}

