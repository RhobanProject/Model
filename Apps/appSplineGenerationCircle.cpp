#include <iostream>
#include <fstream>
#include "Plot/Plot.hpp"
#include "Spline/FittedSpline.hpp"
#include "Spline/SplineContainer.hpp"
#include "Model/SigmabanFloatingModel.hpp"
#include "Model/InverseKinematics.hpp"

/**
 * Generate simple circular center of mass 
 * trajectory for Sigmaban and export them 
 * as fitted polynomial splines
 */
int main()
{
    //Trajectory generation config
    const double configCOMZOffset = 0.01;
    const double configTimePeriod = 2.0;
    const double configCircleRadius = 0.02;

    //Initialize instance
    Leph::Plot plot;
    Leph::SigmabanFloatingModel model;
    Leph::InverseKinematics inverseModel(model);
    
    //Set foot model on zero z
    model.putOnGround();
    //Add allowed degrees of freedom
    inverseModel.addDOF("left foot roll");
    inverseModel.addDOF("left foot pitch");
    inverseModel.addDOF("left knee");
    inverseModel.addDOF("left hip roll");
    inverseModel.addDOF("left hip pitch");
    inverseModel.addDOF("left hip yaw");
    inverseModel.addDOF("right foot roll");
    inverseModel.addDOF("right foot pitch");
    inverseModel.addDOF("right knee");
    inverseModel.addDOF("right hip roll");
    inverseModel.addDOF("right hip pitch");
    inverseModel.addDOF("right hip yaw");
    inverseModel.addDOF("base Tx");
    inverseModel.addDOF("base Tz");
    inverseModel.addDOF("base Ty");
    //Add target constraints
    inverseModel.addTargetPosition("left foot", "left foot tip");
    inverseModel.addTargetPosition("right foot", "right foot tip");
    inverseModel.addTargetOrientation("left foot", "left foot tip");
    inverseModel.addTargetOrientation("right foot", "right foot tip");
    inverseModel.addTargetCOM();
    //Add degrees of freedom bounds 
    inverseModel.setLowerBound("left knee", 0.0);
    inverseModel.setLowerBound("right knee", 0.0);
    //Compute initial COM
    double initCOMZOffset = model.centerOfMass("origin").z();

    //Initialize outputs Vector and fitted splines
    Leph::VectorLabel outputs = model.getDOF();
    Leph::SplineContainer<Leph::FittedSpline> splines;
    for (size_t i=0;i<outputs.size();i++) {
        splines.add(outputs.getLabel(i));
    }

    for (double t=0;t<=configTimePeriod;t+=0.01) {
        //Build COM trajectory
        inverseModel.targetCOM().z() = initCOMZOffset - configCOMZOffset;
        inverseModel.targetCOM().x() = configCircleRadius*cos(2.0*M_PI*t/configTimePeriod);
        inverseModel.targetCOM().y() = configCircleRadius*sin(2.0*M_PI*t/configTimePeriod);

        //Compute inverse kinematics
        for (unsigned int k=0;k<100;k++) {
            inverseModel.run(0.001, 100);
        }
        //Check target error
        if (
            inverseModel.errorPosition("left foot") > 0.001 ||
            inverseModel.errorPosition("right foot") > 0.001 ||
            inverseModel.errorCOM() > 0.001
        ) {
            std::cerr << "InverseKinematics failed to converge" << std::endl;
            return -1;
        }

        //Convertion from radian to degrees
        outputs.mergeInter(model.getDOF());
        outputs.mulOp(180.0/M_PI);

        //Save computed points for spline fitting
        for (size_t i=0;i<outputs.size();i++) {
            splines.get(outputs.getLabel(i)).addPoint(t, outputs(i));
        }
        
        //Plot
        plot.add(outputs);
    }
    plot.plot("index", "all").render();

    //Splines fittings
    for (size_t i=0;i<splines.size();i++) {
        splines.get(outputs.getLabel(i)).fitting(0.3, true);
    }
    
    //Export computed splines
    splines.exportData("/tmp/appSplineGenerationCircle.splines");

    //Plot fitted splines
    for (double t=0;t<configTimePeriod;t+=0.01) {
        for (size_t i=0;i<splines.size();i++) {
            outputs(i) = splines.get(outputs.getLabel(i)).pos(t);
        }
        plot.add(outputs.rename("", "fitted"));
    }
    plot.plot("index", "all").render();

    return 0;
}

