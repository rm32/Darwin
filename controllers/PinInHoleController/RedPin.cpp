#include "RedPin.hpp"

#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <DARwInOPVisionManager.hpp>

#include <webots/Motor.hpp>


// contrstructor to initialise variables
RedPin::RedPin(DarwinRobot* r){
    // set the robot
    robot = r;
    
    // get camera width and heigh
    imageWidth= robot-> getCameraWidth();
    imageHeight = robot-> getCameraWidth();
    
    // get robot managers
    gait= robot->getGaitManager();
    vision =   robot->getVisionManager();
    motion = robot->getMotionManager();
    
    // sets the max and min limb position so the robots head is focused eye level and below
    // ie. looking towards the floor
    int head = 19;
    maxLimbPos[head] = robot-> getMinLimbPos(head) +0.9;
    minLimbPos[head] = robot-> getMinLimbPos(head);
}


// destructor
RedPin::~RedPin()
{
    //TODO
}


// find the relative xAxis and yAxis location of the pin
void RedPin::findLocation()
{
    // gets the image from the robots camera
    image = robot->getImage();
    
    // uses the vision manager to get the pin xAxis and yAxis from the image
    bool find = vision->getBallCenter(xAxis ,yAxis, image);
    
    // if the pin isn't found, xAxis and yAxis are set to 0
    if(!find) {
        xAxis = 0.0;
        yAxis = 0.0;
        isFound = false;
    } else {
        // if the pin is found set the relative xAxis and yAxis.
        // Note. the xAxis is corrected slightly so the robot walks to
        // the left of the pin
        xAxis = 2.0 * xAxis / imageWidth  - 0.186;
        yAxis = 2.0 * yAxis / imageHeight - 1.0;
        isFound = true;
    }
}


// the y value will increase as the robot looks further down towards the floor
// as it gets closer to the pin. When the yAxis is at a certain threshold we
// tell the robot that it has reached the pin
bool RedPin::checkStop(double x, double y)
{
    if(y > 0.35)
    {
        return true;
    }else{
        return false;
    }
}


// robot squats down and touches pin
// TODO: this should be changed to pick up the pin in future
// on a gripper is put into webots
void RedPin::interactWithTarget()
{
    // the robot should stop moving
    gait->stop();
    robot->wait(300);
    
    // squat down
    motion->playPage(15);
    robot->wait(100);
    
    // Robots left arm is extended to the right
    robot->getMotors(5)->setPosition(1.63); // lower arm
    robot->getMotors(1)->setPosition(-0.56);   // upper army
    
    // Robots left leg bend in order to reach the pin
    // The robot then waits 10 seconds in order to give the user time to pick
    // up the pin
    robot->getMotors(7)->setPosition(0.7);   // left leg
    robot->wait(10000);
    
    // The robot returns to its upright position
    motion->playPage(9);
    robot-> wait(200);
}