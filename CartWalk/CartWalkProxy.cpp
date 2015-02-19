#include "CartWalk/CartWalkProxy.hpp"

namespace Leph {

CartWalkProxy::CartWalkProxy() :
    _walk()
{
    //Sigmaban femur and tibia length
    _walk.setLength(9.6, 10.4);
}

double CartWalkProxy::getPhase() const
{
    return _walk.t;
}
        
void CartWalkProxy::setPhase(double phase)
{
    _walk.t = phase;
}

VectorLabel CartWalkProxy::buildOutputs() const
{
    return VectorLabel(
        "output:left hip pitch", 0.0,
        "output:left hip roll", 0.0,
        "output:left hip yaw", 0.0,
        "output:left knee", 0.0,
        "output:left foot pitch", 0.0,
        "output:left foot roll", 0.0,
        "output:right hip pitch", 0.0,
        "output:right hip roll", 0.0,
        "output:right hip yaw", 0.0,
        "output:right knee", 0.0,
        "output:right foot pitch", 0.0,
        "output:right foot roll", 0.0,
        "info:phase", 0.0,
        "info:left spline X", 0.0,
        "info:left spline Y", 0.0,
        "info:left spline Z", 0.0,
        "info:right spline X", 0.0,
        "info:right spline Y", 0.0,
        "info:right spline Z", 0.0
    );
}
VectorLabel CartWalkProxy::buildParams() const
{
    return VectorLabel(
        "static:timeGain", 2.40,
        "static:riseGain", 3.0,
        "static:swingGain", 1.0,
        "static:swingPhase", 0.0,
        "static:swingHeight", 0.0,
        "static:xOffset", 1.5,
        "static:yOffset", 1.5,
        "static:zOffset", 4.0,
        "static:hipOffset", 15.0,
        "static:yLat", 0.0,
        "static:swingForce", 0.1,
        "static:riseRatio", 0.6,
        "static:riseStepPhase", 0.0,
        "dynamic:enabled", 0,
        "dynamic:step", 0.0,
        "dynamic:lateral", 0.0,
        "dynamic:turn", 0.0
    );
}

VectorLabel CartWalkProxy::buildParamsMin() const
{
    return VectorLabel(
        "static:timeGain", 1.0,
        "static:riseGain", 0.0,
        "static:swingGain", 0.0,
        "static:swingPhase", -0.5,
        "static:swingHeight", 0.0,
        "static:xOffset", -5.0,
        "static:yOffset", 0.0,
        "static:zOffset", 0.0,
        "static:hipOffset", -5.0,
        "static:yLat", -2.0,
        "static:swingForce", 0.0,
        "static:riseRatio", 0.1,
        "static:riseStepPhase", 0.0,
        "dynamic:enabled", 0,
        "dynamic:step", -5.0,
        "dynamic:lateral", -12.0,
        "dynamic:turn", -50.0
    );
}
VectorLabel CartWalkProxy::buildParamsMax() const
{
    return VectorLabel(
        "static:timeGain", 3.5,
        "static:riseGain", 6.0,
        "static:swingGain", 4.0,
        "static:swingPhase", 1.5,
        "static:swingHeight", 0.0,
        "static:xOffset", 5.0,
        "static:yOffset", 4.0,
        "static:zOffset", 5.0,
        "static:hipOffset", 25.0,
        "static:yLat", 2.0,
        "static:swingForce", 1.25,
        "static:riseRatio", 0.9,
        "static:riseStepPhase", 0.0,
        "dynamic:enabled", 1,
        "dynamic:step", 16.0,
        "dynamic:lateral", 12.0,
        "dynamic:turn", 50.0
    );
}
        
VectorLabel CartWalkProxy::buildParamsDelta() const
{
    VectorLabel max = buildParamsMax();
    VectorLabel min = buildParamsMin();

    max.subOp(min);
    max.mulOp(1.0/50.0);
    return max;
}
        
VectorLabel CartWalkProxy::exec(
    double deltaTime,
    const VectorLabel& params)
{
    _walk.timeGain = params("static:timeGain");
    _walk.riseGain = params("static:riseGain");
    _walk.swingGain = params("static:swingGain");
    _walk.swingPhase = params("static:swingPhase");
    _walk.swingHeight = params("static:swingHeight");
    _walk.xOffset = params("static:xOffset");
    _walk.yOffset = params("static:yOffset");
    _walk.zOffset = params("static:zOffset");
    _walk.hipOffset = params("static:hipOffset");
    _walk.yLat = params("static:yLat");
    _walk.swingForce = params("static:swingForce");
    _walk.riseRatio = params("static:riseRatio");
    _walk.riseStepPhase = params("static:riseStepPhase");

    _walk.stepGain = params("dynamic:step");
    _walk.lateralStepGain = params("dynamic:lateral");
    _walk.turn = params("dynamic:turn");
    _walk.isEnabled = params("dynamic:enabled");
    
    _walk.tick(deltaTime);
    
    VectorLabel outputs = buildOutputs();
    outputs("output:left hip pitch") = _walk.a_l_hip_pitch;
    outputs("output:left hip roll") = _walk.a_l_hip_roll;
    outputs("output:left hip yaw") = _walk.a_l_hip_yaw;
    outputs("output:left knee") = _walk.a_l_knee;
    outputs("output:left foot pitch") = _walk.a_l_foot_pitch;
    outputs("output:left foot roll") = _walk.a_l_foot_roll;
    outputs("output:right hip pitch") = _walk.a_r_hip_pitch;
    outputs("output:right hip roll") = _walk.a_r_hip_roll;
    outputs("output:right hip yaw") = _walk.a_r_hip_yaw;
    outputs("output:right knee") = _walk.a_r_knee;
    outputs("output:right foot pitch") = _walk.a_r_foot_pitch;
    outputs("output:right foot roll") = _walk.a_r_foot_roll;

    outputs("info:phase") = _walk.t;
    outputs("info:left spline X") = _walk.sLX;
    outputs("info:left spline Y") = _walk.sLY;
    outputs("info:left spline Z") = _walk.sLZ;
    outputs("info:right spline X") = _walk.sRX;
    outputs("info:right spline Y") = _walk.sRY;
    outputs("info:right spline Z") = _walk.sRZ;

    _lastOutputs = outputs;

    return outputs;
}
        
VectorLabel CartWalkProxy::lastOutputs() const
{
    return _lastOutputs;
}

}

