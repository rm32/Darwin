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
    
    // create and initiate the camera
    mCamera = getCamera("Camera");
    mCamera->enable(2*mTimeStep);
    
    //create a display screen
    mDisplay = createDisplay("new_display");
    
    // create and initialise the excelerometer
    mAccelerometer = getAccelerometer("Accelerometer");
    mAccelerometer->enable(mTimeStep);
    
    getGyro("Gyro")->enable(mTimeStep);
    
    keyboardEnable(mTimeStep);
    
    // set up the motors and the sensors
    for (int i=0; i<NMOTORS; i++) {
        mMotors[i] = getMotor(motorNames[i]);
        string sensorName = motorNames[i];
        sensorName.push_back('S');
        mPositionSensors[i] = getPositionSensor(sensorName);
        mPositionSensors[i]->enable(mTimeStep);
        minMotorPositions[i] = mMotors[i]->getMinPosition();
        maxMotorPositions[i] = mMotors[i]->getMaxPosition();
    }
    
    // set up vision manager with a red hue
    mVisionManager = new DARwInOPVisionManager(mCamera->getWidth(), mCamera->getHeight(), 355, 15, 60, 15, 0, 30);
    
    // initialise the motion manager
    mMotionManager = new DARwInOPMotionManager(this);
    
    // initialise the gait manager
    mGaitManager = new DARwInOPGaitManager(this, "config.ini");
    
    setup();
}


DarwinRobot::~DarwinRobot()
{
}


//------ SETUP --------


void DarwinRobot::setup()
{
    faceUp = 0;
    faceDown = 0;
    acc_tolerance = 80.0;
    acc_step = 20;
    
    // First step to update sensors values
    incrementTimestep();
    
    mMotionManager->playPage(9); // init position
    wait(200);
    
    // play the motion preparing the robot to walk
    mGaitManager->start();
    mGaitManager->step(mTimeStep);
    wait(200);
}



// --------- GETTER FUNCTIONS ------------

webots::Motor* DarwinRobot::getMotors(int i)
{
    return mMotors[i];
}

webots::Display* DarwinRobot::getDisplay()
{
    return mDisplay;
}

// --- Get max and min limb positions

double DarwinRobot::getMaxLimbPos(int i)
{
    return maxMotorPositions[i];
}

double DarwinRobot::getMinLimbPos(int i)
{
    return minMotorPositions[i];
}

// ---- Get camera parameters and image ----

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

// Filters the camera image to make it black and white
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


// ------- Gets the Darwin-op managers --------------
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



//-------------- HELPER METHODS -----

// incremement the robot timestep and check if the robot has fallen over
void DarwinRobot::incrementTimestep(){
    checkIfFallen(faceUp, faceDown, acc_tolerance, acc_step);
    int ret = step(mTimeStep);
    if (ret == -1)
    {
        exit(EXIT_SUCCESS);
    }
}


// waits a set number of miliseconds
void DarwinRobot::wait(int ms) {
    double startTime = getTime();
    double s = (double) ms / 1000.0;
    while (s + startTime >= getTime())
    {
        incrementTimestep();
    }
}


// if the value is not in range it is clamped to either the minimum or maximum value
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

//--------------- ROBOT METHODS -----

void DarwinRobot::checkIfFallen(int &fup, int &fdown, const double acc_tolerance, const double acc_step)
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
    if (faceUp > acc_step) {
        mMotionManager->playPage(1); // init position
        mMotionManager->playPage(10); // f_up
        mMotionManager->playPage(9); // walkready position
        faceUp = 0;
    }
    // the back face is down
    else if (faceDown > acc_step) {
        mMotionManager->playPage(1); // init position
        mMotionManager->playPage(11); // b_up
        mMotionManager->playPage(9); // walkready position
        faceDown = 0;
    }
}



// ---------------  TARGET METHODS -----

// search around looking for the target
bool DarwinRobot::findTarget(Target* o){
    // reset values to check if the robot has fallen over
    faceUp = 0;
    faceDown = 0;
    acc_tolerance = 80.0;
    acc_step = 20;
    
    // keep searching till target is found
    while(!o->checkIfFound()){
        // tun round
        mGaitManager->setXAmplitude(0.0);
        mGaitManager->setAAmplitude(0.1);
        mGaitManager->step(mTimeStep);
        
        // move the head vertically
        double headPosition = clamp((0.7*sin(2.0*getTime())), o->getMinLimbPos(19),o->getMaxLimbPos(19));
        mMotors[19]->setPosition(headPosition);
        
        o->findLocation();
        incrementTimestep();
    }
    return true;
}


// walk towards the target
bool DarwinRobot::walkToTarget(Target* o)
{
    bool atTarget = false;
    
    // reset values to check if the robot has fallen over
    faceUp = 0;
    faceDown = 0;
    acc_tolerance = 80.0;
    acc_step = 20;
    
    // keep walking until robot is at the target
    while(!atTarget){
        o->findLocation();
        xAxis = o->getXAxis();
        yAxis = o->getYAxis();
        
        // compute the direction of the head
        // the head move at maximum by 0.015 [rad] at each time step
        xAxis  = 0.015*xAxis + px;
        yAxis  = 0.015*yAxis + py;
        px = xAxis;
        py = yAxis;
        
        double neckPosition =  clamp(-xAxis, minMotorPositions[18], maxMotorPositions[18]);
        double headPosition =  clamp(-yAxis, minMotorPositions[19], maxMotorPositions[19]);
        
        // go forwards and turn according to the head rotation
        mGaitManager->setXAmplitude(1);
        mGaitManager->setAAmplitude(neckPosition);
        mGaitManager->step(mTimeStep);
        
        // Move head
        mMotors[18]->setPosition(neckPosition);
        mMotors[19]->setPosition(headPosition);
        
        // if robot is at the target then stop
        if(o->checkStop(xAxis,yAxis))
        {
            mGaitManager->stop();
            wait(300);
            atTarget = true;
            return true;
        }
        
        incrementTimestep();
    }
    
    return false;
}

