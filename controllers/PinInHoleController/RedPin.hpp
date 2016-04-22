#ifndef REDPIN_HPP
#define REDPIN_HPP

#include "Target.hpp"
#include "DarwinRobot.hpp"

// Red Pin target that the Darwin Robot can interact with

class RedPin : public Target {
    
public:
    RedPin(DarwinRobot* r);
    ~RedPin();
    
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
    
};

#endif