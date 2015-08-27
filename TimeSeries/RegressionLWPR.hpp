#ifndef LEPH_REGRESSIONLWPR_HPP
#define LEPH_REGRESSIONLWPR_HPP

#include <Eigen/Dense>
#include <lwpr_eigen.hpp>
#include "TimeSeries/Regression.hpp"

namespace Leph {

/**
 * RegressionLWPR
 *
 * Implementation of Regression class using
 * LWPR non linear non parametric regression
 * algorithm.
 */
class RegressionLWPR : public Regression
{
    public:

        /**
         * Initialization
         */
        RegressionLWPR() :
            Regression(),
            _model(nullptr)
        {
        }
        
        /**
         * Inherit Optimizable
         */
        virtual inline size_t parameterSize() const override
        {
            return 2*Regression::inputSize() + 5;
        }
        virtual inline MetaParameter defaultParameter
            (size_t index) const override
        {
            size_t size = Regression::inputSize();
            MetaParameter param;
            if (index < size) {
                param = MetaParameter("NormIn", 1.0);
                param.setMinimum(0.0);
            }
            if (index == size) {
                param = MetaParameter("InitAlpha", 50.0);
                param.setMinimum(0.0);
            }
            if (index == size+1) {
                param = MetaParameter("MetaRate", 250.0);
                param.setMinimum(0.0);
            }
            if (index == size+2) {
                param = MetaParameter("Penalty", 1e-6);
                param.setMinimum(0.0);
            }
            if (index >= size+3 && index <= 2*size+2) {
                param = MetaParameter("InitD", 1.0);
                param.setMinimum(0.00001);
            }
            if (index == 2*size+3) {
                param = MetaParameter("WGen", 0.1);
                param.setMinimum(0.0);
                param.setMaximum(0.999);
            }
            if (index == 2*size+4) {
                param = MetaParameter("WPrune", 0.9);
                param.setMinimum(0.0);
            }
            
            return param;
        }
        
        /**
         * Inherit Regression
         */
        virtual inline bool learn(double time) override
        {
            Eigen::VectorXd in = retrieveInputs(time);
            if (
                (size_t)in.size() == inputSize() && 
                Regression::getOutput()->isTimeValid(time)
            ) {
                _model->update(in, Regression::getOutput()->get(time));
                return true;
            } else {
                return false;
            }
        }
        virtual inline double predict(double time) const override
        {
            Eigen::VectorXd in = retrieveInputs(time);
            if ((size_t)in.size() == inputSize()) {
                double yp = _model->predict(in, 0.0)(0);
                return yp;
            } else {
                throw std::runtime_error("RegressionLWPR invalid time");
            }
        }

        /**
         * Initialize an new LWPR model 
         * object using current meta parameters
         * Inherit Regression
         */
        inline virtual void resetRegression() override
        {
            size_t size = Regression::inputSize();

            //InitializeLWPR model with 
            //input dimension
            if (_model != nullptr) {
                delete _model;
            }
            _model = new LWPR_Object(size);
    
            //Normalisation value for each input dimension (>0)
            Eigen::VectorXd vect(size);
            for (size_t i=0;i<size;i++) {
                vect(i) = Optimizable::getParameter(i).value();
            }
            _model->normIn(vect);
            //Use only diagonal matrix. Big speed up in high dimension
            //but lower performance in complex learning.
            _model->diagOnly(true);
            //Learning rate for gradient descente (>0)
            //(meta optimized)
            _model->setInitAlpha(Optimizable::getParameter(size).value());
            //Automatic tunning of distance metric 
            _model->useMeta(true);
            //Meta tunning learning rate (>0)
            _model->metaRate(Optimizable::getParameter(size+1).value());
            //Larger value enforce wider receptive field (>0)
            _model->penalty(Optimizable::getParameter(size+2).value());
            //Set diagonal (input_dim) or complet (input_dim*input_dim)
            //initial distance matrix (>0)
            for (size_t i=0;i<size;i++) {
                vect(i) = Optimizable::getParameter(size+3+i).value();
            }
            _model->setInitD(vect);
            //Receptive field activation threshold (>0)
            _model->wGen(Optimizable::getParameter(2*size+3).value());
            //Receptive field remove threshold
            _model->wPrune(Optimizable::getParameter(2*size+4).value());
        }

        /**
         * Access to internal LWPR model
         * for LWPR related specific query
         */
        inline const LWPR_Object& model() const
        {
            return *_model;
        }
        inline LWPR_Object& model()
        {
            return *_model;
        }

        /**
         * Returned as an Eigen Vector the input
         * at given time with declared lag offset.
         * A zero sized vector is returned if
         * inputs are not all available.
         */
        inline Eigen::VectorXd retrieveInputs(double time) const
        {
            size_t size = Regression::inputSize();
            Eigen::VectorXd vect(size);

            //Parse all registered inputs
            for (size_t i=0;i<size;i++) {
                const TimeSeries* series = Regression::getInput(i).series;
                double delta = Regression::getInput(i).deltaTime;
                if (series->isTimeValid(time-delta)) {
                    vect(i) = series->get(time-delta);
                } else {
                    //No result if asked lagged time is not available
                    return Eigen::VectorXd();
                }
            }

            return vect;
        }

    protected:

        virtual inline void onAddInput() override
        {
            //Reset meta parameters
            Optimizable::resetParameters();
            //Reset the internal LWPR model
            resetRegression();
        }

    private:
    
        /**
         * LWPR learned model
         */
        mutable LWPR_Object* _model;
};

}

#endif

