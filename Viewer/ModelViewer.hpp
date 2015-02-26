#ifndef LEPH_MODELVIEWER_HPP
#define LEPH_MODELVIEWER_HPP

#include <list>
#include <SFML/Window.hpp>
#include <Eigen/Dense>

namespace Leph {

/**
 * ModelViewer
 *
 * Simple OpenGL viewer to
 * display in SFML windows
 */
class ModelViewer
{
    public:

        /**
         * Drawing config
         */
        double frameLength = 1.0;
        double frameThickness = 3.0;
        double groundThickness = 1.0;
        size_t maxTrajectory = 2000;
        
        /**
         * Camera control config
         */
        double camPosVel = 0.002;
        double camViewVel = 0.01;

        /**
         * Initialize SFML and OpenGL rendering
         * with optional windows size
         */
        ModelViewer(unsigned int width = 800, 
            unsigned int height = 600);

        /**
         * Poll event and update screen drawing
         * return false on exit asked
         */
        bool update();

        /**
         * Draw frame
         * Orientation is the transform rotation matrix 
         * from local coordinates to base
         */
        void drawFrame(
            const Eigen::Vector3d& center, 
            const Eigen::Matrix3d& orientation);

        /**
         * Draw mass
         * Orientation is the transform rotation matrix 
         * from local coordinates to base
         */
        void drawMass(
            const Eigen::Vector3d& center,
            const Eigen::Matrix3d& orientation);
        
        /**
         * Draw joint
         * Orientation is the transform rotation matrix 
         * from local coordinates to base
         */
        void drawJoint(
            const Eigen::Vector3d& center, 
            const Eigen::Matrix3d& orientation);

        /**
         * Draw a link between given two points
         */
        void drawLink(
            const Eigen::Vector3d& pt1,
            const Eigen::Vector3d& pt2,
            double colorLevel);

        /**
         * Add a position for drawing trajectory
         */
        void addTrackedPoint(const Eigen::Vector3d& point);

        /**
         * Draw rolling point trajectory
         */
        void drawTrajectory();

    private:

        /**
         * SFML Windows instance
         */
        sf::Window _window;

        /**
         * Camera position and view direction
         */
        Eigen::Vector3d _camPos;
        Eigen::Vector3d _camView;

        /**
         * Rolling history of tracked point trajectory
         */
        std::list<Eigen::Vector3d> _trajectory;

        /**
         * Last recorded mouse position
         */
        int _lastMousePosX;
        int _lastMousePosY;

        /**
         * Update the opengl camera
         */
        void updateCamera();

        /**
         * Convert given rotation matrix to
         * OpenGL and multiply current matrix with
         */
        void applyRotation(const Eigen::Matrix3d& orientation);

        /**
         * Draw ground centered on origin with
         * given size
         */
        void drawGround(double size);

        /**
         * Draw a unit cube with given color
         */
        void drawCube(float r, float g, float b);
};

}

#endif

