
#include "RedPin.hpp"
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <DARwInOPVisionManager.hpp>

#include <webots/Motor.hpp>

RedPin::RedPin(DarwinRobot* r){
    robot = r;
    int head = 19;
    imageWidth= robot-> getCameraWidth();
    imageHeight = robot-> getCameraWidth();
    maxPos[head] = robot-> getMinPos(head) +0.9;
    minPos[head] = robot-> getMinPos(head);
    gait= robot->getGaitManager();
    vision =   robot->getVisionManager();
    motion = robot->getMotionManager();
}

RedPin::~RedPin()
{
}

void RedPin::interactWithTarget()
{
    gait->stop();

    robot->wait(300);
    motion->playPage(15); // squat down
    robot->wait(100);
    robot->getMotors(4)->setPosition(-1.65); // lower arm
    robot->getMotors(0)->setPosition(0.6);   // upper arm
    robot->getMotors(8)->setPosition(0.7);   // right leg
    robot->wait(10000);
    motion->playPage(9); // init position
    robot-> wait(200);
}

void RedPin::findLocation()
{
    
    image = robot->getImage();
    bool find = vision->getBallCenter(x,y, image);
    
    if(!find) {
        x = 0.0;
        y = 0.0;
        isFound = false;
    } else {
        x = 2.0 * x / imageWidth  - 1.5;
        y = 2.0 * y / imageHeight - 1.0;
        isFound = true;
    }
    
}

bool RedPin::checkStop(double x, double y)
{
    if(y > 0.35)
    {
        return true;
    }else{
        return false;
    }
}