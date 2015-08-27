#ifndef LEPH_FOOTSTEPINTEGRATORCONCEPT_HPP
#define LEPH_FOOTSTEPINTEGRATORCONCEPT_HPP

#include "TimeSeries/Concept.hpp"

namespace Leph {

/**
 * FootStepIntegratorConcept
 *
 * Implement an position intregration
 * from delta inputs with respect to footstep.
 * Inputs:
 * 0: delta_x_support_left
 * 1: delta_y_support_left
 * 2: delta_theta_support_left
 * 3: delta_x_support_right
 * 4: delta_y_support_right
 * 5: delta_theta_support_right
 * Outputs:
 * 0: pos_x
 * 1: pos_y
 * 2: pos_theta
 */
class FootStepIntegratorConcept : public Concept
{
    public:
        
        /**
         * Inherit Concept
         */
        virtual std::string name() const override;
        virtual size_t inputSize() const override;
        virtual size_t outputSize() const override;

        /**
         * Inherit Optimize
         */
        virtual size_t parameterSize() const override;
        virtual Leph::MetaParameter defaultParameter
            (size_t index) const override;

    protected:
        
        /**
         * Inherit Concept
         */
        virtual bool doCompute(double time) override;
};

}

#endif

