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
        "timeGain", 2.68,
        "riseGain", 1.5,
        "swingGain", 1.0,
        "swingPhase", -0.05,
        "swingHeight", 0.0,
        "xOffset", 1.5,
        "yOffset", 1.5,
        "zOffset", 2.2,
        "hipOffset", 15.0,
        "yLat", 0.0,
        "swingForce", 0.1,
        "riseRatio", 0.6,
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
        "riseGain", 0.0,
        "swingGain", 0.0,
        "swingPhase", -0.5,
        "swingHeight", 0.0,
        "xOffset", -5.0,
        "yOffset", -1.0,
        "zOffset", 0.0,
        "hipOffset", -5.0,
        "yLat", -3.0,
        "swingForce", 0.0,
        "riseRatio", 0.1,
        "riseStepPhase", 0.0
    );
}
VectorLabel CartWalkProxy::buildStaticParamsMax() const
{
    return VectorLabel(
        "timeGain", 3.5,
        "riseGain", 12.0,
        "swingGain", 6.0,
        "swingPhase", 1.5,
        "swingHeight", 0.0,
        "xOffset", 5.0,
        "yOffset", 3.0,
        "zOffset", 6.0,
        "hipOffset", 25.0,
        "yLat", 3.0,
        "swingForce", 1.25,
        "riseRatio", 0.9,
        "riseStepPhase", 0.0
    );
}
VectorLabel CartWalkProxy::buildDynamicParamsMin() const
{
    return VectorLabel(
        "enabled", 0,
        "step", -10.0,
        "lateral", -25.0,
        "turn", -60.0
    );
}
VectorLabel CartWalkProxy::buildDynamicParamsMax() const
{
    return VectorLabel(
        "enabled", 1,
        "step", 30.0,
        "lateral", 25.0,
        "turn", 60.0
    );
}
        
VectorLabel CartWalkProxy::buildStaticParamsDelta() const
{
    VectorLabel max = buildStaticParamsMax();
    VectorLabel min = buildStaticParamsMin();

    max.vect() -= min.vect();
    max.vect() *= (1.0/50.0);
    return max;
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

