#include "TrajectoryDefinition/CommonTrajs.h"

namespace Leph {

TrajectoryParameters DefaultTrajParameters()
{
    //Default trajectory parameter initialization
    //(No Optimized)
    TrajectoryParameters parameters;
    //CMA-ES parameters
    parameters.add("cmaes_max_iterations", 1000.0);
    parameters.add("cmaes_restarts", 3.0);
    parameters.add("cmaes_lambda", 10.0);
    parameters.add("cmaes_sigma", -1.0);
    //Double support static position
    parameters.add("static_double_pos_trunk_pos_x", 0.009816280388);
    parameters.add("static_double_pos_trunk_pos_y", -0.07149996496);
    parameters.add("static_double_pos_trunk_pos_z", 0.2786133349);
    parameters.add("static_double_pos_trunk_axis_x", 0.0);
    parameters.add("static_double_pos_trunk_axis_y", 0.1919862181);
    parameters.add("static_double_pos_trunk_axis_z", 0.0);
    parameters.add("static_double_pos_foot_pos_x", 0.0);
    parameters.add("static_double_pos_foot_pos_y", -0.1429999995);
    parameters.add("static_double_pos_foot_pos_z", 0.0);
    parameters.add("static_double_pos_foot_axis_x", 0.0);
    parameters.add("static_double_pos_foot_axis_y", 0.0);
    parameters.add("static_double_pos_foot_axis_z", 0.0);
    //Single support static position
    parameters.add("static_single_pos_trunk_pos_x", -0.00557785331559037);
    parameters.add("static_single_pos_trunk_pos_y", -0.0115849568418458);
    parameters.add("static_single_pos_trunk_pos_z", 0.285);
    parameters.add("static_single_pos_trunk_axis_x", -0.672036398746933);
    parameters.add("static_single_pos_trunk_axis_y", 0.0743358280850477);
    parameters.add("static_single_pos_trunk_axis_z", 0.0028323027017884);
    parameters.add("static_single_pos_foot_pos_x", 0.0208647084129351);
    parameters.add("static_single_pos_foot_pos_y", -0.095);
    parameters.add("static_single_pos_foot_pos_z", 0.0591693358237435);
    parameters.add("static_single_pos_foot_axis_x", 0.0);
    parameters.add("static_single_pos_foot_axis_y", 0.0);
    parameters.add("static_single_pos_foot_axis_z", 0.0);

    return parameters;
}

}

