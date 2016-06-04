#ifndef LEPH_MODELDRAW_HPP
#define LEPH_MODELDRAW_HPP

#include "Model/Model.hpp"
#include "Model/HumanoidModel.hpp"
#include "Viewer/ModelViewer.hpp"

namespace Leph {

/**
 * ModelDraw
 *
 * Draw the given Model with given ModelViewer
 */
void ModelDraw(Model& model, ModelViewer& viewer);

/**
 * CameraDraw
 *
 * Draw with given HumanoidModel the camera 
 * projected field of view with given ModelViewer
 */
void CameraDraw(
    const CameraParameters& params, 
    HumanoidModel& model, 
    ModelViewer& viewer);

}

#endif

