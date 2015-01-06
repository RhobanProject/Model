#include "CartWalk/CartWalkGradient.hpp"

namespace Leph {

Matrix CartWalkGradient::differentiation(
    double phase,
    const VectorLabel& staticParams,
    const VectorLabel& dynamicParams,
    double diffStep)
{
    size_t rows = _walk.buildOutputs().size();
    size_t cols = _walk.buildStaticParams().size();
    Matrix jacobian(rows, cols);
    
    VectorLabel tmpStaticParams;
    for (size_t i=0;i<cols;i++) {
        tmpStaticParams = staticParams;
        tmpStaticParams(i) += diffStep;
        _walk.setPhase(phase);
        VectorLabel tmpOutput1 = _walk.exec(0.0, dynamicParams, tmpStaticParams);
        
        tmpStaticParams = staticParams;
        tmpStaticParams(i) -= diffStep;
        _walk.setPhase(phase);
        VectorLabel tmpOutput2 = _walk.exec(0.0, dynamicParams, tmpStaticParams);

        for (size_t j=0;j<rows;j++) {
            jacobian(j, i) = (tmpOutput1(j) - tmpOutput2(j))/(2.0*diffStep);
        }
    }

    return jacobian;
}

}

