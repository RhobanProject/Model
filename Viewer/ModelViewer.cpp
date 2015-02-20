#include <SFML/OpenGL.hpp>
#include "Viewer/ModelViewer.hpp"

namespace Leph {

ModelViewer::ModelViewer(unsigned int width, 
    unsigned int height) :
    _window(sf::VideoMode(width, height), "OpenGL"),
    _camPos(10.0, 10.0, 10.0),
    _camView(-10.0, -10.0, -10.0)
{
    _window.setVerticalSyncEnabled(true);
    _camView.normalize();
    
    //Set color and depth clear value
    glClearDepth(1.0);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    //Enable Z-buffer
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    //Setup projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(
        60.0, (double)width/(double)height, 
        0.0, 1000.0);       
}
        
bool ModelViewer::update()
{
    //Handle keyboard
    sf::Event event;
    while (_window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            return false;
        }
        if (event.type == sf::Event::KeyPressed && 
            event.key.code == sf::Keyboard::Escape
        ) {
            return false;
        }
    }
    //Camera control
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Up) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::Z) 
    ) {
        _camPos += _camPosVel*_camView;
    }
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Down) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::S) 
    ) {
        _camPos -= _camPosVel*_camView;
    }
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Left) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::Q) 
    ) {
        _camPos -= _camPosVel*(
            _camView.cross(Eigen::Vector3d(0.0, 0.0, 1.0)));
    }
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Right) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::D) 
    ) {
        _camPos += _camPosVel*(
            _camView.cross(Eigen::Vector3d(0.0, 0.0, 1.0)));
    }

    //Reset drawing
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Camera
    updateCamera();
    //Drawing
    drawFrame(0.0, 0.0, 0.0);
    
    //Update drawing
    _window.display();
    
    return true;
}
        
void ModelViewer::updateCamera()
{
    Eigen::Vector3d viewPoint = _camPos + _camView;
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        _camPos.x(), _camPos.y(), _camPos.z(), 
        viewPoint.x(), viewPoint.y(), viewPoint.z(), 
        0.0, 0.0, 1.0);
}
        
void ModelViewer::drawFrame(double x, double y, double z)
{
    glLineWidth(2.5);
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(1.0, 0.0, 0.0);
    glEnd();
    glBegin(GL_LINES);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 1.0, 0.0);
    glEnd();
    glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 1.0);
    glEnd();
}

}

