#include <stdexcept>
#include "Utils/IterativeLearningControl.hpp"

namespace Leph {

IterativeLearningControl::IterativeLearningControl(
    size_t lag, double learningRate) :
    _learningRate(learningRate),
    _lag(lag),
    _container(),
    _currentOffsets(),
    _nextOffsets()
{
}
        
VectorLabel IterativeLearningControl::getOffsets(double phase) const
{
    if (_container.size() == 0 && _currentOffsets.size() == 0) {
        throw std::logic_error("IterativeLearningControl empty");
    }

    VectorLabel offsets = _container.back();
    for (size_t i=0;i<offsets.size();i++) {
        offsets(i) = _currentOffsets[i].pos(phase);
    }

    return offsets;
}
        
const VectorLabel& IterativeLearningControl::getErrors() const
{
    return _pastErrors;
}

void IterativeLearningControl::update(double phase, 
    const VectorLabel& goals, 
    const VectorLabel& motors)
{
    //Check size
    if (motors.size() != goals.size()) {
        throw std::logic_error(
            "IterativeLearningControl invalid size");
    }
    //Initialize splines container
    if (_currentOffsets.size() == 0) {
        _pastErrors = motors;
        _pastErrors.zeroOp();
        _currentErrors = motors;
        _currentErrors.zeroOp();
        for (size_t i=0;i<goals.size();i++) {
            _currentOffsets.push_back(LinearSpline());
            _nextOffsets.push_back(LinearSpline());
            _currentOffsets.back().addPoint(0.0, 0.0);
            _currentOffsets.back().addPoint(1.0, 0.0);
        }
    } else if (_currentOffsets.size() != goals.size()) {
        throw std::logic_error(
            "IterativeLearningControl invalid size");
    }

    //Store motors goals positions
    _container.push_back(goals);
    while (_container.size() > _lag) {
        _container.pop_front();
    }

    //No learning if the history is not completed
    if (_container.size() != _lag) {
        return;
    }

    //Swap splines on phase cycling
    if (phase < _nextOffsets.front().max()) {
        _currentOffsets = _nextOffsets;
        for (size_t i=0;i<_nextOffsets.size();i++) {
            _nextOffsets[i] = LinearSpline();
        }
        _pastErrors = _currentErrors;
        _currentErrors.zeroOp();
    }

    //Compute motors error
    VectorLabel errors = motors;
    errors.subOp(_container.front());
    _currentErrors.addOp(errors);
    errors.mulOp(-_learningRate);
    errors.addOp(getOffsets(phase));
    for (size_t i=0;i<errors.size();i++) {
        _nextOffsets[i].addPoint(phase, errors(i));
    }
}
        
}

