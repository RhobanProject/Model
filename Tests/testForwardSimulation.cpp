#include <iostream>
#include "Model/HumanoidModel.hpp"
#include "Model/ForwardSimulation.hpp"
#include "Viewer/ModelViewer.hpp"
#include "Viewer/ModelDraw.hpp"
#include "Model/RBDLRootUpdate.h"
#include "Plot/Plot.hpp"

void testForward()
{
    //Simple pendulum
    Leph::Model model("../Data/pendulum_triple.urdf");
    model.setDOF("roll1", M_PI/2.0);
    model.setDOF("roll2", -0.8);
    
    //Test empty constraints
    Leph::RBDL::ConstraintSet constraints;
    constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
    constraints.Bind(model.getRBDLModel());

    //Simulator
    Leph::ForwardSimulation sim(model);

    Leph::ModelViewer viewer(1200, 900);
    Leph::Plot plot;
    double t = 0.0;
    size_t count = 0;
    while (viewer.update()) {
        for (int k=0;k<10;k++) {
            t += 0.0001;
            std::cout << "********** count=" << count << std::endl;
            sim.update(0.0001);
            count++;
            Leph::VectorLabel vect;
            for (size_t i=0;i<model.sizeDOF();i++) {
                vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
                vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
                vect.append("output:" + model.getDOFName(i), sim.outputTorques()(i));
                vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
                vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
                vect.append("active:" + model.getDOFName(i), sim.actives()(i));
                vect.append("input:" + model.getDOFName(i), sim.inputTorques()(i));
                vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
                vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
            }
            plot.add(vect);
        }
        model.setDOFVect(sim.positions());
        Leph::ModelDraw(model, viewer);
    }
    plot.plot("index", "acc:*").render();
    plot.plot("index", "active:*").render();
    plot.plot("index", "output:*").render();
    plot
        .plot("index", "output:*")
        .plot("index", "friction:*")
        .plot("index", "control:*")
        .render();
    plot.plot("index", "input:*").render();
    plot.plot("index", "pos:*").render();
    plot.plot("index", "vel:*").render();
    plot.plot("index", "goal:*").render();
}

void testConstraints()
{
    //Load model from URDF file
    std::string urdfFile = "../Data/pendulum_triple.urdf";
    Leph::RBDL::Model modelOld;
    if (!Leph::RBDL::Addons::URDFReadFromFile(
        urdfFile.c_str(), &modelOld, false)
    ) {
        return;
    }
    //Select new RBDL body id root
    size_t frameRootId = 0;
    //Update old urdf model with new root frame
    Leph::RBDL::Model modelNew = 
        Leph::RBDLRootUpdate(modelOld, frameRootId, true);
    //Initialize base model
    //Simple pendulum
    Leph::Model model(modelNew);
    model.setDOF("roll1", M_PI/2.0);
    model.setDOF("roll2", -0.8);

    //Test empty constraints
    Leph::RBDL::ConstraintSet constraints;
    constraints.SetSolver(Leph::RBDLMath::LinearSolverFullPivHouseholderQR);
    //Z 0
    constraints.AddConstraint(
        model.frameIndexToBodyId(model.getFrameIndex("base2")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
    //X 1
    constraints.AddConstraint(
        model.frameIndexToBodyId(model.getFrameIndex("base2")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
    //Y 2
    constraints.AddConstraint(
        model.frameIndexToBodyId(model.getFrameIndex("base2")),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 1.0, 0.0));
    //Z 3
    constraints.AddConstraint(
        model.frameIndexToBodyId(model.getFrameIndex("base2")),
        Leph::RBDLMath::Vector3d(0.1, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
    //X 4
    constraints.AddConstraint(
        model.frameIndexToBodyId(model.getFrameIndex("base2")),
        Leph::RBDLMath::Vector3d(0.1, 0.0, 0.0),
        Leph::RBDLMath::Vector3d(1.0, 0.0, 0.0));
    //Z 5
    constraints.AddConstraint(
        model.frameIndexToBodyId(model.getFrameIndex("base2")),
        Leph::RBDLMath::Vector3d(0.0, 0.1, 0.0),
        Leph::RBDLMath::Vector3d(0.0, 0.0, 1.0));
    constraints.Bind(model.getRBDLModel());

    //Simulator
    Leph::ForwardSimulation sim(model);

    Leph::ModelViewer viewer(1200, 900);
    Leph::Plot plot;
    double t = 0.0;
    size_t count = 0;
    while (viewer.update()) {
        for (int k=0;k<10;k++) {
            t += 0.0001;
            std::cout << "********** count=" << count << std::endl;
            sim.update(0.0001, &constraints);
            count++;
            Leph::VectorLabel vect;
            for (size_t i=0;i<model.sizeDOF();i++) {
                vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
                vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
                vect.append("output:" + model.getDOFName(i), sim.outputTorques()(i));
                vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
                vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
                vect.append("active:" + model.getDOFName(i), sim.actives()(i));
                vect.append("input:" + model.getDOFName(i), sim.inputTorques()(i));
                vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
                vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
            }
            vect.append("force:1", constraints.force(0));
            vect.append("force:2", constraints.force(3));
            vect.append("force:3", constraints.force(5));
            plot.add(vect);
        }
        //if (count >= 160) break;
        model.setDOFVect(sim.positions());
        Leph::ModelDraw(model, viewer);
    }
    plot.plot("index", "force:*").render();
    plot.plot("index", "acc:*").render();
    plot.plot("index", "active:*").render();
    plot.plot("index", "output:*").render();
    plot
        .plot("index", "output:*")
        .plot("index", "friction:*")
        .plot("index", "control:*")
        .render();
    plot.plot("index", "input:*").render();
    plot.plot("index", "pos:*").render();
    plot.plot("index", "vel:*").render();
    plot.plot("index", "goal:*").render();
}

void testHumanoid()
{
    //Full humanoid
    Leph::HumanoidModel model(
        Leph::SigmabanModel, 
        "left_foot_tip", false);
    
    //Simulator
    Leph::ForwardSimulation sim(model);

    Leph::ModelViewer viewer(1200, 900);
    Leph::Plot plot;
    double t = 0.0;
    size_t count = 0;
    while (viewer.update()) {
        for (int k=0;k<10;k++) {
            t += 0.0001;
            std::cout << "********** count=" << count << std::endl;
            sim.update(0.0001);
            count++;
            Leph::VectorLabel vect;
            for (size_t i=0;i<model.sizeDOF();i++) {
                vect.append("goal:" + model.getDOFName(i), sim.goals()(i));
                vect.append("acc:" + model.getDOFName(i), sim.accelerations()(i));
                vect.append("output:" + model.getDOFName(i), sim.outputTorques()(i));
                vect.append("friction:" + model.getDOFName(i), sim.frictionTorques()(i));
                vect.append("control:" + model.getDOFName(i), sim.controlTorques()(i));
                vect.append("active:" + model.getDOFName(i), sim.actives()(i));
                vect.append("input:" + model.getDOFName(i), sim.inputTorques()(i));
                vect.append("pos:" + model.getDOFName(i), sim.positions()(i));
                vect.append("vel:" + model.getDOFName(i), sim.velocities()(i));
            }
            plot.add(vect);
        }
        model.setDOFVect(sim.positions());
        Leph::ModelDraw(model, viewer);
    }
    plot.plot("index", "acc:*").render();
    plot.plot("index", "active:*").render();
    plot.plot("index", "output:*").render();
    plot
        .plot("index", "output:*")
        .plot("index", "friction:*")
        .plot("index", "control:*")
        .render();
    plot.plot("index", "input:*").render();
    plot.plot("index", "pos:*").render();
    plot.plot("index", "vel:*").render();
    plot.plot("index", "goal:*").render();
}

int main()
{
    testForward();
    testConstraints();
    testHumanoid();
}

