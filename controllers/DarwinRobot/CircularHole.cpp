#include "Hole.hpp"
#include <webots/Display.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <DARwInOPVisionManager.hpp>

#include <webots/Motor.hpp>

#define WB_IMAGE_RGB  3

using namespace cv;

Hole::Hole(DarwinRobot* r){
    robot = r;
    int head = 19;
    imageWidth= robot-> getCameraWidth();
    imageHeight = robot-> getCameraHeight();
    maxPos[head] = robot-> getMaxPos(head);
    minPos[head] = robot-> getMaxPos(head) -0.3;
    gait= robot->getGaitManager();
    vision =   robot->getVisionManager();
    motion = robot->getMotionManager();
}


Hole::~Hole()
{
}


int lastRadius;

bool Hole::checkStop(double x, double y)
{
    if(holeLost > 90){
        return true;
    }
    
    return false;
}

void Hole::findLocation()
{
    webots::Display *mDisplay =   robot->getDisplay();
    
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

void Hole::circleDetection(Mat &image)
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
        lastRadius = radius;
        // circle center
        circle(image, center, 3, Scalar(0, 0, 255), -1, 8, 0);
        // circle outline
        circle(image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }
    
    
    if(circles.size() == 0) {
        holeLost++;
        isFound = false;
    }else {
        holeLost = 0;
        x = 4.0 * center.x / imageWidth  - 1;
        y = 2.0 * center.y / imageHeight - 1.7;
        
        
        if( (y > (-0.6))  && (y < 0))
        {
            y = 2.0 * center.y / imageHeight + 4;
        }else if( y < 1.3)
        {
            y = 2.0 * center.y / imageHeight -20;
        }
        
        if(x < -1)
        {
            x = 4.0 * center.x / imageWidth  + 5;
        }else if( x < 0)
        {
            x = 4.0 * center.x / imageWidth  - 1.2;
        }
        
        isFound = true;
    }
    
}


void Hole::interactWithTarget()
{
    gait->stop();
    robot->wait(300);
    
    
    robot->getMotors(5)->setPosition(0.8);
    robot->getMotors(1)->setPosition(-1);
    
    robot->getMotors(4)->setPosition(-0.8); // lower arm
    robot->getMotors(0)->setPosition(1);   // upper arm
}
