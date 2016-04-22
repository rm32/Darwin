#ifndef CIRCULARHOLE_HPP
#define CIRCULARHOLE_HPP

#include "Target.hpp"
#include "DarwinRobot.hpp"

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

// Circular Hole object that the Darwin robot can interact with

class CircularHole : public Target {
public:
    CircularHole(DarwinRobot* r);
    ~CircularHole();
    
    // finds the location of the target object and updates the xAxis and yAxis
    virtual void findLocation();
    
    // sets a boolean to true when an robot walking towards the target should stop
    // given the xAxis and yAxis
    virtual bool checkStop(double x, double y);
    
    // allows the robot to interact with the target in a specific way related to
    // the target.
    virtual void interactWithTarget();
    
private:
    // stores the robot
    DarwinRobot* robot;
    
    // stores the filtered image
    Mat filteredImage;
    
    // stores the last found circle radius
    int lastFoundRadius;
    
    // counts how long since a circle was found
    int CircularHoleLost;
    
    // filters an image
    Mat filter(Mat &s);
    
    // applies circle detection on an image
    void circleDetection(Mat &s);

};

#endif