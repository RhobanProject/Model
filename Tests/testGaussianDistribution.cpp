#include <iostream>
#include <cassert>
#include "Utils/GaussianDistribution.hpp"
#include "Plot/Plot.hpp"

int main()
{
    Leph::Plot plot;
    std::default_random_engine engine;

    Eigen::VectorXd mean1(1);
    Eigen::VectorXd point1(1);
    mean1 << 1.0;
    Eigen::MatrixXd cov1(1, 1);
    cov1 << 0.5;
    Leph::GaussianDistribution dist1(mean1, cov1);
    assert(dist1.mean().size() == 1);
    assert(dist1.mean()(0) == 1.0);
    assert(dist1.covariance().rows() == 1);
    assert(dist1.covariance().cols() == 1);
    assert(dist1.covariance()(0, 0) == 0.5);

    for (double x=-4.0;x<=4.0;x+=0.02) {
        point1 << x;
        plot.add({
            "x", x,
            "probability", dist1.probability(point1),
            "log_probability", dist1.logProbability(point1),
        });
    }
    for (size_t k=0;k<100;k++) {
        point1 = dist1.sample(engine);
        plot.add({
            "x", point1(0),
            "points", 0.1,
        });
    }
    plot
        .plot("x", "probability")
        .plot("x", "points", Leph::Plot::Points, "index")
        .render();
    plot.plot("x", "log_probability").render();
    plot.clear();

    Eigen::VectorXd mean2(2);
    Eigen::VectorXd point2(2);
    mean2 << 0.0, 0.0;
    Eigen::MatrixXd cov2(2, 2);
    cov2 << 
        0.5, 0.4,
        0.4, 1.5;
    Leph::GaussianDistribution dist2(mean2, cov2);
    assert(dist2.mean().size() == 2);
    assert(dist2.mean()(0) == 0.0);
    assert(dist2.mean()(1) == 0.0);
    assert(dist2.covariance().rows() == 2);
    assert(dist2.covariance().cols() == 2);
    assert(dist2.covariance()(0, 0) == 0.5);
    assert(dist2.covariance()(0, 1) == 0.1);
    assert(dist2.covariance()(1, 0) == 0.1);
    assert(dist2.covariance()(1, 1) == 0.8);

    for (double x=-4.0;x<=4.0;x+=0.1) {
        for (double y=-4.0;y<=4.0;y+=0.1) {
            point2 << x, y;
            plot.add({
                "x", x,
                "y", y,
                "probability", dist2.probability(point2),
            });
        }
    }
    std::vector<Eigen::VectorXd> points;
    for (size_t k=0;k<1000;k++) {
        point2 = dist2.sample(engine);
        points.push_back(point2);
        plot.add({
            "x", point2(0),
            "y", point2(1),
            "points", 0.05,
        });
    }
    plot
        .plot("x", "y", "probability", Leph::Plot::Points, "probability")
        .plot("x", "y", "points", Leph::Plot::Points)
        .render();
    plot.clear();

    for (size_t j=0;j<500;j++) {
        for (size_t k=0;k<1000;k++) {
            point2 = dist2.sample(engine);
            points.push_back(point2);
        }
        Leph::GaussianDistribution dist3;
        dist3.fit(points);
        std::cout 
            << "Size=" << points.size()
            << " MeanError=" 
            << (dist3.mean()-dist2.mean()).norm()
            << " CovError=" 
            << (dist3.covariance()-dist2.covariance()).norm() << std::endl;
    }

    return 0;
}

