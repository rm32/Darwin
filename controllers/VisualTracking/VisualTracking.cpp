#include "VisualTracking.hpp"
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <DARwInOPVisionManager.hpp>

#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/PositionSensor.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

VisualTracking::VisualTracking(): Robot() {
  mTimeStep = getBasicTimeStep();
  
  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mCamera = getCamera("Camera");
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


}

VisualTracking::~VisualTracking() {
}

void VisualTracking::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void VisualTracking::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void VisualTracking::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;
  
  // count how many steps the accelerometer
  // says that the robot is down
  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;
  
  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;
  
  // the robot face is down
  if (fup > acc_step) {
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9); // init position    
    fup = 0;
  }
  // the back face is down
  else if (fdown > acc_step) {
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9); // init position
    fdown = 0;
  }
}

// function containing the main feedback loop
void VisualTracking::run() {


  double horizontal = 0.0;
  double vertical = 0.0;
  int width  = mCamera->getWidth();
  int height = mCamera->getHeight();

  cout << "---------------Visual Tracking---------------" << endl;
  cout << "This example illustrates the possibilities of Vision Manager." << endl;
  cout << "Move the red ball by dragging the mouse while keeping both shift key and mouse button pressed." << endl;
	
  // First step to update sensors values
  myStep();

  // play the hello motion
  mMotionManager->playPage(9); // init position
  wait(200);
  
  //mGaitManager->start(); // init walking mode 
        
  wait(200);
  
  
  while (true) {
    checkIfFallen();
    
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    
    double x, y;
    
    bool ballInFieldOfView = mVisionManager->getBallCenter(x, y, mCamera->getImage());

    
    // Eye led indicate if ball has been found
    if(ballInFieldOfView)
      mEyeLED->set(0x00FF00);
    else
      mEyeLED->set(0xFF0000);
     
    if(!ballInFieldOfView)
    {
     mGaitManager->start(); // init walking mode 
     mGaitManager->setAAmplitude(-0.5);
    }else{
      mGaitManager->stop();
    }
 
    
    // Move the head in direction of the ball if it has been found
    if(ballInFieldOfView) {
      double dh = 0.1*((x / width ) - 0.5);
      horizontal -= dh;
      double dv = 0.1*((y / height ) - 0.5);
      vertical -= dv;
      horizontal = clamp(horizontal, minMotorPositions[18], maxMotorPositions[18]);
      horizontal = clamp(horizontal, minMotorPositions[19], maxMotorPositions[19]);
      mMotors[18]->setPosition(horizontal);
      mMotors[19]->setPosition(vertical);
    }
    
    mGaitManager->step(mTimeStep);

    // step
    myStep();
  }
  }
  
  
  