// webot classes
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>

// Darwin classes
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include <DARwInOPVisionManager.hpp>

// custom classes
#include "DarwinRobot.hpp"
#include "RedPin.hpp"
#include "Hole.hpp"

//openCV classes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;

// Darwin Robot constructor
DarwinRobot::DarwinRobot(): Robot() {
    mTimeStep = getBasicTimeStep();
    
    static const char *motorNames[NMOTORS] = {
        "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
        "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
        "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
        "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
        "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
    };
    
    mCamera = getCamera("Camera");
    mDisplay = createDisplay("new_display");
    
    mCamera->enable(2*mTimeStep);
    
    for (int i=0; i<NMOTORS; i++) {
        mMotors[i] = getMotor(motorNames[i]);
        string sensorName = motorNames[i];
        sensorName.push_back('S');
        mPositionSensors[i] = getPositionSensor(sensorName);
        mPositionSensors[i]->enable(mTimeStep);
        minMotorPositions[i] = mMotors[i]->getMinPosition();
        maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    }
    
    mVisionManager = new DARwInOPVisionManager(mCamera->getWidth(), mCamera->getHeight(), 355, 15, 60, 15, 0, 30);
    
    mAccelerometer = getAccelerometer("Accelerometer");
    mAccelerometer->enable(mTimeStep);
    
    getGyro("Gyro")->enable(mTimeStep);
    
    keyboardEnable(mTimeStep);
    
    mMotionManager = new DARwInOPMotionManager(this);
    mGaitManager = new DARwInOPGaitManager(this, "config.ini");
    
    setup();
    
    
}


DarwinRobot::~DarwinRobot()
{
}


// --------- GETTER FUNCTIONS ------------

int DarwinRobot::getCameraWidth()
{
    return mCamera->getWidth();
}

int DarwinRobot::getCameraHeight()
{
    return mCamera->getHeight();
}


const unsigned char* DarwinRobot::getImage()
{
    return mCamera->getImage();
}



// TODO: WANT TO MOVE THIS FUNCTION TO PIN
Mat DarwinRobot::getMat()
{
    static int width  = mCamera->getWidth();
    static int height = mCamera->getHeight();
    
    // get camera image
    Mat img = Mat(Size(width, height), CV_8UC4);
    img.data= (uchar*) getImage();
    
    cvtColor(img, img, COLOR_BGRA2GRAY);
    
    return img;
    
}



webots::Display* DarwinRobot::getDisplay()
{
    return mDisplay;
}


managers::DARwInOPVisionManager* DarwinRobot::getVisionManager()
{
    return mVisionManager;
}

managers::DARwInOPGaitManager* DarwinRobot::getGaitManager()
{
    return mGaitManager;
}

managers::DARwInOPMotionManager* DarwinRobot::getMotionManager()
{
    return mMotionManager;
}


void DarwinRobot::updateSensorValues(){
    checkIfFallen(fup, fdown, acc_tolerance, acc_step);
    int ret = step(mTimeStep);
    if (ret == -1)
    {
        exit(EXIT_SUCCESS);
    }
}



void DarwinRobot::wait(int ms) {
    double startTime = getTime();
    double s = (double) ms / 1000.0;
    while (s + startTime >= getTime())
    {
        updateSensorValues();
    }
}



double DarwinRobot::clamp(double value, double min, double max)
{
    if(value < min)
    {
        return min;
    }else if(value > max)
    {
        return max;
    }else{
        return value;
    }
    
}

bool DarwinRobot::checkIfFallen(int &fup, int &fdown, const double acc_tolerance, const double acc_step)
{    
    const double *acc = mAccelerometer->getValues();
    // count how many steps the accelerometer
    // says that the robot is down
    if (acc[1] < 512.0 - acc_tolerance)
    {
        fup++;
    } else{
        fup = 0;
    }
    
    if (acc[1] > 512.0 + acc_tolerance){
        fdown++;
    }else{
        fdown = 0;
    }
        
    // the robot face is down
    if (fup > acc_step) {
      mMotionManager->playPage(1); // init position
      mMotionManager->playPage(10); // f_up
      mMotionManager->playPage(9); // walkready position
      fup = 0;
    }
    // the back face is down
    else if (fdown > acc_step) {
      mMotionManager->playPage(1); // init position
      mMotionManager->playPage(11); // b_up
      mMotionManager->playPage(9); // walkready position
      fdown = 0;
    }
    else{
        return false;
    }
}

bool DarwinRobot::findItem(Item* o){
     fup = 0;
     fdown = 0;
     acc_tolerance = 80.0;
     acc_step = 20;
         printf("reset val 2 \n");
    while(!o->checkIfFound()){
        // tun round
        mGaitManager->setXAmplitude(0.0);
        mGaitManager->setAAmplitude(0.1);
        mGaitManager->step(mTimeStep);
        
        // move the head vertically
        double headPosition = clamp((0.7*sin(2.0*getTime())), o->getMinPos(19),o->getMaxPos(19));
        mMotors[19]->setPosition(headPosition);
        
        o->findLocation();
        updateSensorValues();
    }
    return true;
}

bool DarwinRobot::walkToItem(Item* o)
{
    bool atItem = false;
     fup = 0;
     fdown = 0;
     acc_tolerance = 80.0;
     acc_step = 20;
         printf("reset val 3 \n");
    while(!atItem){
        o->findLocation();
        x = o->getX();
        y = o->getY();
        
        // compute the direction of the head
        // the head move at maximum by 0.015 [rad] at each time step
        x  = 0.015*x + px;
        y  = 0.015*y + py;
        px = x;
        py = y;
        
        double neckPosition =  clamp(-x, minMotorPositions[18], maxMotorPositions[18]);
        double headPosition =  clamp(-y, minMotorPositions[19], maxMotorPositions[19]);
        
        // go forwards and turn according to the head rotation
        mGaitManager->setXAmplitude(1);
        mGaitManager->setAAmplitude(neckPosition);
        mGaitManager->step(mTimeStep);
        
        // Move head
        mMotors[18]->setPosition(neckPosition);
        mMotors[19]->setPosition(headPosition);
        
        if(o->checkStop(x,y))
        {
            mGaitManager->stop();
            wait(300);
            atItem = true;
            return true;
        }
        
        updateSensorValues();
    }
    
    return false;
}


double DarwinRobot::getMaxPos(int i)
{
    return maxMotorPositions[i];
}

double DarwinRobot::getMinPos(int i)
{
    return minMotorPositions[i];
}


void DarwinRobot::setup()
{
     fup = 0;
     fdown = 0;
     acc_tolerance = 80.0;
     acc_step = 20;
     
     printf("reset val 1 \n");
    notSet = true;
    // First step to update sensors values
    updateSensorValues();
    
    mMotionManager->playPage(9); // init position
    wait(200);
    
    // play the motion preparing the robot to walk
    mGaitManager->start();
    mGaitManager->step(mTimeStep);
    wait(200);
}

webots::Motor* DarwinRobot::getMotors(int i)
{
  return mMotors[i];
}



int main(int argc, char **argv)
{
    DarwinRobot* controller = new DarwinRobot();
    RedPin* pin = new RedPin(controller);
    controller->findItem(pin);
    controller->walkToItem(pin);
    pin->interactWithTarget();
    controller->setup();
    Hole* hole = new Hole(controller);
    controller->findItem(hole);
    controller->walkToItem(hole);
    hole->interactWithTarget();
    delete pin;
    delete hole;
    delete controller;
    return 0;
}
