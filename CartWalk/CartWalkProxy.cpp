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
    return VectorLabel({
        std::make_pair("phase", 0.0),
        std::make_pair("left hip pitch", 0.0),
        std::make_pair("left hip roll", 0.0),
        std::make_pair("left hip yaw", 0.0),
        std::make_pair("left knee", 0.0),
        std::make_pair("left foot pitch", 0.0),
        std::make_pair("left foot roll", 0.0),
        std::make_pair("left arm", 0.0),
        std::make_pair("right hip pitch", 0.0),
        std::make_pair("right hip roll", 0.0),
        std::make_pair("right hip yaw", 0.0),
        std::make_pair("right knee", 0.0),
        std::make_pair("right foot pitch", 0.0),
        std::make_pair("right foot roll", 0.0),
        std::make_pair("right arm", 0.0)
    });
}
VectorLabel CartWalkProxy::buildStaticParams() const
{
    return VectorLabel({
        std::make_pair("timeGain", 2.75),
        std::make_pair("riseGain", 2.7),
        std::make_pair("swingGain", 0.4),
        std::make_pair("swingPhase", 0.0),
        std::make_pair("swingHeight", 0.0),
        std::make_pair("armsGain", 0.0),
        std::make_pair("xOffset", -0.7),
        std::make_pair("yOffset", 1.0),
        std::make_pair("zOffset", 4.0),
        std::make_pair("yLat", 0.0),
        std::make_pair("swingForce", 0.0),
        std::make_pair("riseRatio", 0.5),
        std::make_pair("riseStepPhase", 0.0)
    });
}
VectorLabel CartWalkProxy::buildDynamicParams() const
{
    return VectorLabel({
        std::make_pair("enabled", 0),
        std::make_pair("step", 0.0),
        std::make_pair("lateral", 0.0),
        std::make_pair("turn", 0.0)
    });
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
    _walk.armsGain = staticParams("armsGain");
    _walk.xOffset = staticParams("xOffset");
    _walk.yOffset = staticParams("yOffset");
    _walk.zOffset = staticParams("zOffset");
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
    outputs("phase") = _walk.t;
    outputs("left hip pitch") = _walk.a_l_hip_pitch;
    outputs("left hip roll") = _walk.a_l_hip_roll;
    outputs("left hip yaw") = _walk.a_l_hip_yaw;
    outputs("left knee") = _walk.a_l_knee;
    outputs("left foot pitch") = _walk.a_l_foot_pitch;
    outputs("left foot roll") = _walk.a_l_foot_roll;
    outputs("left arm") = _walk.a_l_arm;
    outputs("right hip pitch") = _walk.a_r_hip_pitch;
    outputs("right hip roll") = _walk.a_r_hip_roll;
    outputs("right hip yaw") = _walk.a_r_hip_yaw;
    outputs("right knee") = _walk.a_r_knee;
    outputs("right foot pitch") = _walk.a_r_foot_pitch;
    outputs("right foot roll") = _walk.a_r_foot_roll;
    outputs("right arm") = _walk.a_r_arm;

    return outputs;
}

}

