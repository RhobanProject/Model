#include <SFML/OpenGL.hpp>
#include "Viewer/ModelViewer.hpp"

namespace Leph {

ModelViewer::ModelViewer(unsigned int width, 
    unsigned int height) :
    _window(sf::VideoMode(width, height), "OpenGL", 
        sf::Style::Default, sf::ContextSettings(32)),
    _camPos(0.0, 0.0, 0.0),
    _camView(1.0, 0.0, 0.0),
    _trajectoryRed(),
    _trajectoryGreen(),
    _trajectoryBlue(),
    _trajectoryYellow(),
    _trajectoryPurple(),
    _trajectoryCyan(),
    _lastMousePosX(0),
    _lastMousePosY(0)
{
    _window.setVerticalSyncEnabled(true);
    _camView.normalize();
    
    //Setup projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(
        60.0, (double)width/(double)height, 
        0.01, 1000.0);       
    //Enable Z-buffer
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthRange(0.0f, 1.0f);
    //Set color and depth clear value
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClearDepth(1.0f);
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
    //Handle mouse relative motion
    sf::Vector2i mousePosition = sf::Mouse::getPosition();
    double mouseDeltaPosX = mousePosition.x - _lastMousePosX;
    double mouseDeltaPosY = mousePosition.y - _lastMousePosY;
    _lastMousePosX = mousePosition.x;
    _lastMousePosY = mousePosition.y;
    //Camera view control
    Eigen::Vector3d camLat = _camView.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
    Eigen::Vector3d camUp = _camView.cross(camLat);
    _camView = Eigen::AngleAxisd(-camViewVel*mouseDeltaPosY, camLat)
        .toRotationMatrix()*_camView;
    _camView = Eigen::AngleAxisd(camViewVel*mouseDeltaPosX, camUp)
        .toRotationMatrix()*_camView;
    _camView.normalize();
    //Camera position control
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Up) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::Z) 
    ) {
        _camPos += camPosVel*_camView;
    }
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Down) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::S) 
    ) {
        _camPos -= camPosVel*_camView;
    }
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Left) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::Q) 
    ) {
        _camPos -= camPosVel*camLat;
    }
    if (
        sf::Keyboard::isKeyPressed(sf::Keyboard::Right) || 
        sf::Keyboard::isKeyPressed(sf::Keyboard::D) 
    ) {
        _camPos += camPosVel*camLat;
    }

    //Camera
    updateCamera();
    //Drawing
    drawGround(frameLength*20.0);
    
    //Update drawing
    _window.display();
    //Reset drawing
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    return true;
}
        
bool ModelViewer::isKeyPressed(sf::Keyboard::Key key)
{
    return sf::Keyboard::isKeyPressed(key); 
}
        
void ModelViewer::drawFrame(
    const Eigen::Vector3d& center, 
    const Eigen::Matrix3d& orientation)
{
    glPushMatrix();
    glTranslated(center.x(), center.y(), center.z());
    applyRotation(orientation.transpose());
    glLineWidth(frameThickness);
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(frameLength, 0.0, 0.0);
    glEnd();
    glPushMatrix();
        glTranslated(frameLength, 0.0, 0.0);
        glScaled(frameLength/20.0, frameLength/20.0, frameLength/20.0);
        drawCube(1.0, 0.0, 0.0);
    glPopMatrix();
    glBegin(GL_LINES);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, frameLength, 0.0);
    glEnd();
    glPushMatrix();
        glTranslated(0.0, frameLength, 0.0);
        glScaled(frameLength/20.0, frameLength/20.0, frameLength/20.0);
        drawCube(0.0, 1.0, 0.0);
    glPopMatrix();
    glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, frameLength);
    glEnd();
    glPushMatrix();
        glTranslated(0.0, 0.0, frameLength);
        glScaled(frameLength/20.0, frameLength/20.0, frameLength/20.0);
        drawCube(0.0, 0.0, 1.0);
    glPopMatrix();
    glLineWidth(frameThickness/4.0);
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(-frameLength, 0.0, 0.0);
    glEnd();
    glBegin(GL_LINES);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, -frameLength, 0.0);
    glEnd();
    glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, -frameLength);
    glEnd();
    glPopMatrix();
}
        
void ModelViewer::drawMass(const Eigen::Vector3d& center,
    const Eigen::Matrix3d& orientation)
{
    glPushMatrix();
        glTranslated(center.x(), center.y(), center.z());
        applyRotation(orientation.transpose());
        glScaled(frameLength/4.0, frameLength/4.0, frameLength/4.0);
        drawCube(0.6, 0.3, 0.3);
    glPopMatrix();
}
        
void ModelViewer::drawJoint(
    const Eigen::Vector3d& center, 
    const Eigen::Matrix3d& orientation)
{
    glPushMatrix();
        glTranslated(center.x(), center.y(), center.z());
        applyRotation(orientation.transpose());
        glLineWidth(frameThickness);
        glBegin(GL_LINES);
            glColor3f(0.3, 0.3, 0.6);
            glVertex3f(-2.0*frameLength, 0.0, 0.0);
            glVertex3f(2.0*frameLength, 0.0, 0.0);
        glEnd();
        glScaled(3.0*frameLength/4.0, frameLength/6.0, frameLength/6.0);
        drawCube(0.3, 0.3, 0.6);
    glPopMatrix();
}
        
void ModelViewer::drawLink(
    const Eigen::Vector3d& pt1,
    const Eigen::Vector3d& pt2)
{
    double dist = (pt1-pt2).norm();
    Eigen::Vector3d vect = pt2 - pt1;

    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axis = vect.cross(Eigen::Vector3d(1.0, 0.0, 0.0));
    if (axis.norm() > 0.001) {
        double angle = vect.dot(Eigen::Vector3d(1.0, 0.0, 0.0))/(vect.norm());
        angle = acos(angle);
        axis.normalize();
        transform = Eigen::AngleAxisd(-angle, axis).toRotationMatrix();
    } else if (vect.x() < 0.0) {
        transform = -transform;
    }

    glPushMatrix();
        glTranslated(pt1.x(), pt1.y(), pt1.z());
        applyRotation(transform);
        glScaled(dist/2.0, frameLength/10.0, frameLength/10.0);
        glTranslated(1.0, 0.0, 0.0);
        drawCube(0.3, 0.6, 0.3);
    glPopMatrix();
}

void ModelViewer::drawBox(double sizeX, double sizeY, double sizeZ,
    const Eigen::Vector3d& center, 
    const Eigen::Matrix3d& orientation,
    double r, double g, double b)
{
    glPushMatrix();
        glTranslated(center.x(), center.y(), center.z());
        applyRotation(orientation.transpose());
        glScaled(sizeX, sizeY, sizeZ);
        glLineWidth(frameThickness);
        drawCube(r, g, b, true);
    glPopMatrix();
}
        
void ModelViewer::addTrackedPoint(const Eigen::Vector3d& point, 
    Color color)
{
    if (color == Red) {
        _trajectoryRed.push_front(point);
        while (_trajectoryRed.size() > maxTrajectory) {
            _trajectoryRed.pop_back();
        }
    }
    if (color == Green) {
        _trajectoryGreen.push_front(point);
        while (_trajectoryGreen.size() > maxTrajectory) {
            _trajectoryGreen.pop_back();
        }
    }
    if (color == Blue) {
        _trajectoryBlue.push_front(point);
        while (_trajectoryBlue.size() > maxTrajectory) {
            _trajectoryBlue.pop_back();
        }
    }
    if (color == Yellow) {
        _trajectoryYellow.push_front(point);
        while (_trajectoryYellow.size() > maxTrajectory) {
            _trajectoryYellow.pop_back();
        }
    }
    if (color == Purple) {
        _trajectoryPurple.push_front(point);
        while (_trajectoryPurple.size() > maxTrajectory) {
            _trajectoryPurple.pop_back();
        }
    }
    if (color == Cyan) {
        _trajectoryCyan.push_front(point);
        while (_trajectoryCyan.size() > maxTrajectory) {
            _trajectoryCyan.pop_back();
        }
    }
}
void ModelViewer::drawTrajectory()
{
    drawColorTrajectory(_trajectoryRed, 1.0, 0.0, 0.0);
    drawColorTrajectory(_trajectoryGreen, 0.0, 1.0, 0.0);
    drawColorTrajectory(_trajectoryBlue, 0.0, 0.0, 1.0);
    drawColorTrajectory(_trajectoryYellow, 1.0, 1.0, 0.0);
    drawColorTrajectory(_trajectoryPurple, 1.0, 0.0, 1.0);
    drawColorTrajectory(_trajectoryCyan, 0.0, 1.0, 1.0);
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
        
void ModelViewer::applyRotation(const Eigen::Matrix3d& orientation)
{
    GLfloat m[16];
    m[0] = orientation(0, 0);
    m[1] = orientation(1, 0);
    m[2] = orientation(2, 0);
    m[3] = 0.0;
    m[4] = orientation(0, 1);
    m[5] = orientation(1, 1);
    m[6] = orientation(2, 1);
    m[7] = 0.0;
    m[8] = orientation(0, 2);
    m[9] = orientation(1, 2);
    m[10] = orientation(2, 2);
    m[11] = 0.0;
    m[12] = 0.0;
    m[13] = 0.0;
    m[14] = 0.0;
    m[15] = 1.0;
    glMultMatrixf(m);
}
        
void ModelViewer::drawGround(double size)
{
    glLineWidth(groundThickness);
    for (double x=-size;x<=size;x+=2.0*frameLength) {
        glBegin(GL_LINES);
            glColor3f(1.0, 1.0, 1.0);
            glVertex3f(x, -size, 0.0);
            glVertex3f(x, size, 0.0);
        glEnd();
    }
    for (double y=-size;y<=size;y+=2.0*frameLength) {
        glBegin(GL_LINES);
            glColor3f(1.0, 1.0, 1.0);
            glVertex3f(-size, y, 0.0);
            glVertex3f(size, y, 0.0);
        glEnd();
    }
}
        
void ModelViewer::drawCube(float r, float g, float b,
    bool isWireFrame)
{
    if (isWireFrame) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    glBegin(GL_QUADS);
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(1.0, 1.0, -1.0);
        glVertex3f(1.0, -1.0, -1.0);
        glVertex3f(1.0, -1.0, 1.0);
        
        glColor3f(r, g, b);
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, -1.0);
        glVertex3f(-1.0, -1.0, -1.0);
        glVertex3f(-1.0, -1.0, 1.0);
        
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(-1.0, -1.0, 1.0);
        glVertex3f(1.0, -1.0, 1.0);
        
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, -1.0);
        glVertex3f(-1.0, 1.0, -1.0);
        glVertex3f(-1.0, -1.0, -1.0);
        glVertex3f(1.0, -1.0, -1.0);
        
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, -1.0);
        glVertex3f(1.0, 1.0, -1.0);

        glColor3f(r, g, b);
        glVertex3f(1.0, -1.0, 1.0);
        glVertex3f(-1.0, -1.0, 1.0);
        glVertex3f(-1.0, -1.0, -1.0);
        glVertex3f(1.0, -1.0, -1.0);
    glEnd();
    if (isWireFrame) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}
        
void ModelViewer::drawColorTrajectory(
    const std::list<Eigen::Vector3d>& traj, 
    double r, double g, double b)
{
    if (traj.size() == 0) {
        return;
    }

    //Draw first point as big point
    glPointSize(frameThickness*5.0);
    glBegin(GL_POINTS);
        glColor3f(r, g, b);
        glVertex3f(
            traj.front().x(), 
            traj.front().y(), 
            traj.front().z());
    glEnd();

    //Draw remaining trajectory as lines
    Eigen::Vector3d oldPt = traj.front();
    size_t length = 1;
    for (const Eigen::Vector3d& pt : traj) {
        double rr = r - (double)length/maxTrajectory;
        double gg = g - (double)length/maxTrajectory;
        double bb = b - (double)length/maxTrajectory;
        if (rr <= 0.0) rr = 0.0;
        if (gg <= 0.0) gg = 0.0;
        if (bb <= 0.0) bb = 0.0;
        if (rr == 0.0 && gg == 0.0 && bb == 0.0) break;
        glLineWidth(frameThickness*10.0);
        glBegin(GL_LINES);
            glColor3f(rr, gg, bb);
            glVertex3f(oldPt.x(), oldPt.y(), oldPt.z());
            glVertex3f(pt.x(), pt.y(), pt.z());
        glEnd();
        oldPt = pt;
        length++;
    }
}
        
}

