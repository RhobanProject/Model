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
        "left hip pitch", 0.0,
        "left hip roll", 0.0,
        "left hip yaw", 0.0,
        "left knee", 0.0,
        "left foot pitch", 0.0,
        "left foot roll", 0.0,
        "right hip pitch", 0.0,
        "right hip roll", 0.0,
        "right hip yaw", 0.0,
        "right knee", 0.0,
        "right foot pitch", 0.0,
        "right foot roll", 0.0
    );
}
VectorLabel CartWalkProxy::buildInfo() const
{
    return VectorLabel(
        "phase", 0.0,
        "left spline X", 0.0,
        "left spline Y", 0.0,
        "left spline Z", 0.0,
        "right spline X", 0.0,
        "right spline Y", 0.0,
        "right spline Z", 0.0
    );
}
VectorLabel CartWalkProxy::buildStaticParams() const
{
    return VectorLabel(
        "timeGain", 2.35,
        "riseGain", 1.7,
        "swingGain", 0.7,
        "swingPhase", 0.35,
        "swingHeight", 0.0,
        "xOffset", -0.05,
        "yOffset", 0.65,
        "zOffset", 2.2,
        "hipOffset", 20.5,
        "yLat", 0.65,
        "swingForce", 0.8,
        "riseRatio", 0.8,
        "riseStepPhase", 0.0
    );
}
VectorLabel CartWalkProxy::buildDynamicParams() const
{
    return VectorLabel(
        "enabled", 0,
        "step", 0.0,
        "lateral", 0.0,
        "turn", 0.0
    );
}

VectorLabel CartWalkProxy::buildStaticParamsMin() const
{
    return VectorLabel(
        "timeGain", 1.0,
        "riseGain", 1.0,
        "swingGain", 0.0,
        "swingPhase", 0.0,
        "swingHeight", 0.0,
        "xOffset", -0.7,
        "yOffset", 1.0,
        "zOffset", 4.0,
        "hipOffset", 0.0,
        "yLat", 0.0,
        "swingForce", 0.0,
        "riseRatio", 0.5,
        "riseStepPhase", 0.0
    );
}
VectorLabel CartWalkProxy::buildStaticParamsMax() const
{
    return VectorLabel(
        "timeGain", 3.0,
        "riseGain", 4.0,
        "swingGain", 2.0,
        "swingPhase", 7.0,
        "swingHeight", 0.0,
        "xOffset", -0.7,
        "yOffset", 1.0,
        "zOffset", 4.0,
        "hipOffset", 0.0,
        "yLat", 0.0,
        "swingForce", 0.0,
        "riseRatio", 0.5,
        "riseStepPhase", 0.0
    );
}
VectorLabel CartWalkProxy::buildDynamicParamsMin() const
{
    return VectorLabel(
        "enabled", 0,
        "step", -4.0,
        "lateral", -10.0,
        "turn", -40.0
    );
}
VectorLabel CartWalkProxy::buildDynamicParamsMax() const
{
    return VectorLabel(
        "enabled", 1,
        "step", 10.0,
        "lateral", 10.0,
        "turn", 40.0
    );
}
        
VectorLabel CartWalkProxy::exec(
    double deltaTime,
    const VectorLabel& dynamicParams, 
    const VectorLabel& staticParams)
{
    _walk.timeGain = staticParams("timeGain");
    _walk.riseGain = staticParams("riseGain");
    _walk.swingGain = staticParams("swingGain");
    _walk.swingPhase = staticParams("swingPhase");
    _walk.swingHeight = staticParams("swingHeight");
    _walk.xOffset = staticParams("xOffset");
    _walk.yOffset = staticParams("yOffset");
    _walk.zOffset = staticParams("zOffset");
    _walk.hipOffset = staticParams("hipOffset");
    _walk.yLat = staticParams("yLat");
    _walk.swingForce = staticParams("swingForce");
    _walk.riseRatio = staticParams("riseRatio");
    _walk.riseStepPhase = staticParams("riseStepPhase");

    _walk.stepGain = dynamicParams("step");
    _walk.lateralStepGain = dynamicParams("lateral");
    _walk.turn = dynamicParams("turn");
    _walk.isEnabled = dynamicParams("enabled");
    
    _walk.tick(deltaTime);
    
    VectorLabel outputs = buildOutputs();
    outputs("left hip pitch") = _walk.a_l_hip_pitch;
    outputs("left hip roll") = _walk.a_l_hip_roll;
    outputs("left hip yaw") = _walk.a_l_hip_yaw;
    outputs("left knee") = _walk.a_l_knee;
    outputs("left foot pitch") = _walk.a_l_foot_pitch;
    outputs("left foot roll") = _walk.a_l_foot_roll;
    outputs("right hip pitch") = _walk.a_r_hip_pitch;
    outputs("right hip roll") = _walk.a_r_hip_roll;
    outputs("right hip yaw") = _walk.a_r_hip_yaw;
    outputs("right knee") = _walk.a_r_knee;
    outputs("right foot pitch") = _walk.a_r_foot_pitch;
    outputs("right foot roll") = _walk.a_r_foot_roll;

    VectorLabel info = buildInfo();
    info("phase") = _walk.t;
    info("left spline X") = _walk.sLX;
    info("left spline Y") = _walk.sLY;
    info("left spline Z") = _walk.sLZ;
    info("right spline X") = _walk.sRX;
    info("right spline Y") = _walk.sRY;
    info("right spline Z") = _walk.sRZ;

    _lastOutputs = outputs;
    _lastInfo = info;

    return outputs;
}
        
VectorLabel CartWalkProxy::lastOutputs() const
{
    return _lastOutputs;
}
VectorLabel CartWalkProxy::lastInfo() const
{
    return _lastInfo;
}

}

