#include "Model/SigmabanFixedModel.hpp"

namespace Leph {

SigmabanFixedModel::SigmabanFixedModel() :
    _supportFoot(LeftSupportFoot),
    _modelLeft("left foot tip"),
    _modelRight("right foot tip")
{
}
        
SigmabanFixedModel::SupportFoot SigmabanFixedModel::
    getSupportFoot() const
{
    return _supportFoot;
}
        
const SigmabanModel& SigmabanFixedModel::get() const
{
    if (_supportFoot == LeftSupportFoot) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}
SigmabanModel& SigmabanFixedModel::get()
{
    if (_supportFoot == LeftSupportFoot) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}
        
void SigmabanFixedModel::updateBase()
{
    //Check if moving foot is touching
    if (_supportFoot == LeftSupportFoot) {
        Eigen::Vector3d posFoot = 
            _modelLeft.position("right foot tip", "origin");
        if (posFoot.z() < 0.0) {
            Eigen::Matrix3d rotation = 
                _modelLeft.orientation("right foot tip", "origin").transpose();
            _modelRight.importDOF(_modelLeft);
            _modelRight.setDOF("base Tx", posFoot.x());
            _modelRight.setDOF("base Ty", posFoot.y());
            _modelRight.setDOF("base yaw", atan2(rotation(1, 0), rotation(0, 0)));
            _supportFoot = RightSupportFoot;
        }
    } else {
        Eigen::Vector3d posFoot = 
            _modelRight.position("left foot tip", "origin");
        if (posFoot.z() < 0.0) {
            Eigen::Matrix3d rotation = 
                _modelRight.orientation("left foot tip", "origin").transpose();
            _modelLeft.importDOF(_modelRight);
            _modelLeft.setDOF("base Tx", posFoot.x());
            _modelLeft.setDOF("base Ty", posFoot.y());
            _modelLeft.setDOF("base yaw", atan2(rotation(1, 0), rotation(0, 0)));
            _supportFoot = LeftSupportFoot;
        }
    }
}

}

