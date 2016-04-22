#ifndef PININHOLECONTROLLER_HPP
#define PININHOLECONTROLLER_HPP

#include "DarwinRobot.hpp"
#include "RedPin.hpp"
#include "CircularHole.hpp"

class PinInHoleController {
  public:
    PinInHoleController();
    ~PinInHoleController();
    void run();
  private:
    DarwinRobot* robot;
    CircularHole* hole; 
    RedPin* pin; 
};

#endif