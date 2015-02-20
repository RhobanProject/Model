#ifndef LEPH_MODELVIEWER_HPP
#define LEPH_MODELVIEWER_HPP

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

    private:

        /**
         * Camera control config
         */
        const double _camPosVel = 0.1;

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
         * Update the opengl camera
         */
        void updateCamera();

        /**
         * Draw frame
         */
        void drawFrame(double x, double y, double z);
};

}

#endif

