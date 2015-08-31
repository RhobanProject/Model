#include <cmath>
#include "Concepts/FootStepIntegratorConcept.hpp"

namespace Leph {


std::string FootStepIntegratorConcept::name() const
{
    return "FootStepIntegratorConcept";
}
size_t FootStepIntegratorConcept::inputSize() const
{
    return 6;
}
size_t FootStepIntegratorConcept::outputSize() const
{
    return 3;
}

size_t FootStepIntegratorConcept::parameterSize() const
{
    return 0;
}
Leph::MetaParameter FootStepIntegratorConcept::defaultParameter
    (size_t index) const
{
    (void)index;
    return MetaParameter();
}

bool FootStepIntegratorConcept::doCompute(double time)
{
    //Check that inputs are available
    for (size_t i=0;i<inputSize();i++) {
        if (!Concept::getInput(i)->isTimeValid(time)) {
            return false;
        }
    }

    //Retrieve last outputs value
    double stateX = 0.0;
    double stateY = 0.0;
    double stateTheta = 0.0;
    if (Concept::getOutput(0)->size() != 0) {
        stateX = Concept::getOutput(0)->lastValue();
    }
    if (Concept::getOutput(1)->size() != 0) {
        stateY = Concept::getOutput(1)->lastValue();
    }
    if (Concept::getOutput(2)->size() != 0) {
        stateTheta = Concept::getOutput(2)->lastValue();
    }

    //Find the last support foot
    size_t indexLeft = Concept::getInput(0)->getClosestIndex(time);
    size_t indexRight = Concept::getInput(3)->getClosestIndex(time);
    double timeLeft = Concept::getInput(0)->at(indexLeft).time;
    double timeRight = Concept::getInput(3)->at(indexRight).time;

    //Retrieve delta
    double deltaX;
    double deltaY;
    double deltaTheta;
    double t;
    if (fabs(timeLeft - time) < TIME_EPSILON) {
        deltaX = Concept::getInput(0)->get(time);
        deltaY = Concept::getInput(1)->get(time);
        deltaTheta = Concept::getInput(2)->getAngular(time);
        t = timeLeft;
    } else if (fabs(timeRight - time) < TIME_EPSILON) {
        deltaX = Concept::getInput(3)->get(time);
        deltaY = Concept::getInput(4)->get(time);
        deltaTheta = Concept::getInput(5)->getAngular(time);
        t = timeRight;
    } else {
        return false;
    }
    
    //Check that outputs are available
    for (size_t i=0;i<outputSize();i++) {
        if (
            Concept::getOutput(i)->size() > 0 &&
            Concept::getOutput(i)->timeMax() > t
        ) {
            return false;
        }
    }

    //Compute integration
    double tmpX = deltaX*cos(stateTheta) - deltaY*sin(stateTheta);
    double tmpY = deltaX*sin(stateTheta) + deltaY*cos(stateTheta);
    stateX += tmpX;
    stateY += tmpY;
    stateTheta += deltaTheta;

    //Write new integrated state
    Concept::getOutput(0)->append(t, stateX);
    Concept::getOutput(1)->append(t, stateY);
    Concept::getOutput(2)->append(t, stateTheta);

    return true;
}

}

