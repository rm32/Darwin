#ifndef REDPIN_HPP
#define REDPIN_HPP

#include "Item.hpp"
#include "DarwinRobot.hpp"

class RedPin : public Item {
    
public:
    RedPin(DarwinRobot* c);
    ~RedPin();
    virtual void findLocation();
    virtual bool checkStop(double x, double y);
    virtual void interactWithTarget();
    
private:
    DarwinRobot* robot;
    
};

#endif