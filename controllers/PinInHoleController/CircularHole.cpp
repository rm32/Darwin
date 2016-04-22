#include "CircularHole.hpp"
#include <webots/Display.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <DARwInOPVisionManager.hpp>

#include <webots/Motor.hpp>

#define WB_IMAGE_RGB  3

using namespace cv;

// contrstructor to initialise variables
CircularHole::CircularHole(DarwinRobot* r){
    robot = r;
    int head = 19;
    imageWidth= robot-> getCameraWidth();
    imageHeight = robot-> getCameraHeight();
    maxLimbPos[head] = robot-> getMaxLimbPos(head);
    minLimbPos[head] = robot-> getMaxLimbPos(head) -0.3;
    gait= robot->getGaitManager();
    vision =   robot->getVisionManager();
    motion = robot->getMotionManager();
}


// destructor
CircularHole::~CircularHole()
{
    //TODO
}


// camera image is filtered using open cv bluring and circle detection
// is applied in order to find the relative xAxis and yAxis of the circle
// the filtered image with the found circle drawn on it is then displayed in a display screen.
void CircularHole::findLocation()
{
    webots::Display *mDisplay =   robot->getDisplay();
    
    // The image is already converted to gray scale when receiving it from darwin
    Mat filteredImage = robot->getMat();
    
    
    blur(filteredImage, filteredImage, Size(7, 7));
    GaussianBlur(filteredImage, filteredImage, Size(9, 9), 1, 1);
    
    circleDetection(filteredImage);
    
    // convert image back to RGB so it can be shown in display
    cvtColor(filteredImage, filteredImage, COLOR_GRAY2RGB);
    
    image=  filteredImage.data;
    
    webots::ImageRef *i = mDisplay->imageNew(imageWidth, imageHeight, image, WB_IMAGE_RGB);
    
    mDisplay->imagePaste(i, 0, 0);
}


// circle detection applied on the filtered image
// TODO: circle detection tends to be unstable, especially when robot is too close
// the hole to detect a circle or at and angle to the hole.
// Better method of getting xAxis and yAxis is needed 
void CircularHole::circleDetection(Mat &image)
{
    vector<Vec3f> circles;
    Point center(0, 0);
    
    
    /// Apply the Hough Transform to find the circles
    HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows/4, 23,23);
    /// Draw the circles detected
    for (size_t i = 0; i < circles.size(); i++)
    {
        center.x = cvRound(circles[i][0]);
        center.y = cvRound(circles[i][1]);
        int radius = cvRound(circles[i][2]);
        lastFoundRadius = radius;
        // circle center
        circle(image, center, 3, Scalar(0, 0, 255), -1, 8, 0);
        // circle outline
        circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }
    
    if(circles.size() == 0) {
        CircularHoleLost++;
        isFound = false;
    }else {
        // Attemping to get the xAxis and yAxis and making adjustments when
        // circle is going out of scope
        CircularHoleLost = 0;
        xAxis = 4.0 * center.x / imageWidth  - 1;
        yAxis = 2.0 * center.y / imageHeight - 1.7;
        
        
        if( (yAxis > (-0.6))  && (yAxis < 0))
        {
            yAxis = 2.0 * center.y / imageHeight + 4;
        }else if( yAxis < 1.3)
        {
            yAxis = 2.0 * center.y / imageHeight -20;
        }
        
        if(xAxis < -1)
        {
            xAxis = 4.0 * center.x / imageWidth  + 5;
        }else if( xAxis < 0)
        {
            xAxis = 4.0 * center.x / imageWidth  - 1.2;
        }
        
        isFound = true;
    }
}

// robot stops when at the pin
// TODO: This is currently change wether this works, more solid method needed
// in the future.
bool CircularHole::checkStop(double x, double y)
{
    if(CircularHoleLost > 90){
        return true;
    }
    
    return false;
}


// robot puts arm through the hole
void CircularHole::interactWithTarget()
{
    // robot stop walking
    gait->stop();
    robot->wait(300);
    
    // right atm is fully extended out in front of the robot
    robot->getMotors(5)->setPosition(0.8);  // lower arm
    robot->getMotors(1)->setPosition(-1);   // upper arm
    
    // left arm is fully extended out in front of the robot
    robot->getMotors(4)->setPosition(-0.8); // lower arm
    robot->getMotors(0)->setPosition(1);   // upper arm
}
