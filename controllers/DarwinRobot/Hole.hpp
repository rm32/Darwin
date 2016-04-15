#ifndef HOLE_HPP
#define HOLE_HPP

#include "Item.hpp"
#include "DarwinRobot.hpp"
#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;

class Hole : public Item {
public:
    Hole(DarwinRobot* c);
    ~Hole();
    virtual void findLocation();
    virtual bool checkStop(double x, double y);
    virtual void interactWithTarget();
    
private:
    DarwinRobot* robot;
    Mat filteredImage;
    Mat filter(Mat &s);
    void circleDetection(Mat &s);
};

#endif