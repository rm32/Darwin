// File:          PinInHoleController.cpp
// Date:          21 April 2015   
// Description:   Robot should find pin and touch it, it should
//                then find the hole and put it arm throught the hole
// Author:        Roshenac Mitchell

#include "PinInHoleController.hpp"

PinInHoleController::PinInHoleController()
{
    robot = new DarwinRobot();
    pin = new RedPin(robot);
    hole = new CircularHole(robot);
}


PinInHoleController::~PinInHoleController()
{
    delete pin;
    delete hole;
    delete robot;
}

void PinInHoleController::run()
{ 
    robot->findTarget(pin);
    robot->walkToTarget(pin);
    pin->interactWithTarget();
    
    robot->setup();
    
    robot->findTarget(hole);
    robot->walkToTarget(hole);
    hole->interactWithTarget();
}


// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
int main(int argc, char **argv)
{
  PinInHoleController* controller = new PinInHoleController();
  controller->run();
  delete controller;
  return 0;
}
