#include "VisualTracking.hpp"
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <DARwInOPVisionManager.hpp>
#include <webots/Display.hpp>

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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define WB_IMAGE_RGB  3
#define WB_IMAGE_RGBA 4
#define WB_IMAGE_ARGB 5
#define WB_IMAGE_BGRA 6


using namespace webots;
using namespace managers;
using namespace std;
using namespace cv;

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

static const char *motorNames[NMOTORS] = {
    "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
    "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
    "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
    "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
    "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};


// set up robot
VisualTracking::VisualTracking(): Robot() {
    mTimeStep = getBasicTimeStep();
    
    mEyeLED = getLED("EyeLed");
    mHeadLED = getLED("HeadLed");
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
    
    
}

// destructor
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


static double clamp(double value, double min, double max) {
    if (min > max) {
        assert(0);
        return value;
    }
    return value < min ? min : value > max ? max : value;
}


void VisualTracking::lookAround(double headPosition, double min, double max, double speed)
{
    // set eye led to red
    mEyeLED->set(0xFF0000);
    
    // turn round
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(speed);
    mGaitManager->step(mTimeStep);
    // move the head vertically
    headPosition = clamp(0.7*sin(2.0*getTime()), min,max);
    mMotors[19]->setPosition(headPosition);
}


int VisualTracking::walkTowardsItem(double &x, double &y, double &px, double &py, double &neckPosition, double &headPosition, double offset)
{
    // set eye led to blue
    mEyeLED->set(0x0000FF);
    
    // compute the direction of the head
    // the head move at maximum by 0.015 [rad] at each time step
    x  = 0.015*x + px;
    y  = 0.015*y + py;
    px = x;
    py = y;
    neckPosition = clamp(-x, minMotorPositions[18], maxMotorPositions[18]);
    headPosition = clamp(-y, minMotorPositions[19], maxMotorPositions[19]);
    
    // go forwards and turn according to the head rotation
    if (y < 0.1) // ball far away, go quickly
        mGaitManager->setXAmplitude(0.3);
    else // ball close, go slowly
        mGaitManager->setXAmplitude(0.3);
    
    mGaitManager->setAAmplitude(neckPosition + offset);
    mGaitManager->step(mTimeStep);
    
    // Move head
    mMotors[18]->setPosition(neckPosition);
    mMotors[19]->setPosition(headPosition);

    return y;
}



//*****************
//METHOD TO CHECK TO SEE IF HE HAS FALLEN
//***************

bool VisualTracking::checkIfFallen(int fup, int fdown, const double acc_tolerance, const double acc_step)
{
    const double *acc = mAccelerometer->getValues();
    
    // count how many steps the accelerometer
    // says that the robot is down
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
        mMotionManager->playPage(1); // init position
        wait(50);
        mMotionManager->playPage(10); // f_up
        wait(50);
        mMotionManager->playPage(9); // walkready position
        wait(50);
        fup = 0;
        return true;
    }
    // the back face is down
    else if (fdown > acc_step) {
        mMotionManager->playPage(1); // init position
        wait(50);
        mMotionManager->playPage(11); // b_up
        wait(50);
        mMotionManager->playPage(9); // walkready position
        wait(50);
        fdown = 0;
        return true;
    }
    else{
        return false;
    }
}



//*****************
// METHOD TO FIND BALL
//*****************


bool VisualTracking::getBallCenter(double &x, double &y) {
    static int width  = mCamera->getWidth();
    static int height = mCamera->getHeight();
    
    const unsigned char *im = mCamera->getImage();
    bool find = mVisionManager->getBallCenter(x, y, im);
    
    if(!find) {
        x = 0.0;
        y = 0.0;
        return false;
    } else {
        x = 2.0 * x / width  - 1.0;
        y = 2.0 * y / height - 1.0;
        return true;
    }
}



bool VisualTracking::walkTowardsBall(double &x, double &y, double &px, double &py, double &neckPosition, double &headPosition)
{
    double val = walkTowardsItem(x, y, px, py, neckPosition, headPosition, 0.6);

    
    // if the ball is close enough
    // squat next to the ball
    // higher val number ->  close to ball 
    if (val > 0.3) {
        mGaitManager->stop();
        wait(300);
        mMotionManager->playPage(15); // squat down
        wait(10000);
        return true; 
    }
    
    return false; 
}



//*********************************************
// METHOD FOR FILTER
//*********************************************
Mat filter(Mat &src)
{
    
    // Mat to hold the filtered image
    Mat filterSrc;
    
    /// Initialize arguments for the filter
    Point anchor = Point(-1, -1);
    
    /// Kernel size for a normalized box filter
    int kernel_size = 9;
    
    /// Convert the image to Gray
    cvtColor(src, filterSrc, COLOR_BGRA2GRAY);
    
    // smoothes out the image - Normalized Block Filter
    blur(filterSrc, filterSrc, Size(kernel_size, kernel_size), anchor);
    
    //smoothes out the image - Gaussian Filter
    //GaussianBlur(filterSrc, filterSrc, Size(kernel_size, kernel_size), 2, 2);
    return filterSrc;
}


Point circleDetection(Mat &filtered, Mat &orig)
{
    vector<Vec3f> circles;
    Point center(0, 0);
    
    /// Apply the Hough Transform to find the circles
    HoughCircles(filtered, circles, CV_HOUGH_GRADIENT, 1, 2, 76, 60, 0, 0);
      /// Draw the circles detected
     for (size_t i = 0; i < circles.size(); i++)
     {
     center.x = cvRound(circles[i][0]);
     center.y = cvRound(circles[i][1]);
     int radius = cvRound(circles[i][2]);
     // circle center
     circle(orig, center, 3, Scalar(0, 0, 255), -1, 8, 0);
     // circle outline
     circle(orig, center, radius, Scalar(0, 0, 255), 3, 8, 0);
     }
     
    return center;
}





//*********************
// METHODS TO FIND HOLE
//****************


Point VisualTracking::applyFilter()
{
    static int width  = mCamera->getWidth();
    static int height = mCamera->getHeight();
    
    // get camera image
    Mat img = Mat(Size(width, height), CV_8UC4);
    img.data= (uchar*) mCamera->getImage();
    
    Mat filteredImage = filter(img);
    Point p = circleDetection(filteredImage , filteredImage) ;
    
    // convert image back to RGB so it can be shown in display
    cvtColor(filteredImage, filteredImage, COLOR_GRAY2RGB);
    
    const unsigned char *image=  filteredImage.data;
    
    ImageRef *i = mDisplay->imageNew(width, height, image, WB_IMAGE_RGB);
    
    mDisplay->imagePaste(i, 0, 0);
    
    return p;
}


bool VisualTracking::walkTowardsHole(double &x, double &y, double &px, double &py, double &neckPosition, double &headPosition)
{

   double val = walkTowardsItem(x, y, px, py, neckPosition, headPosition, 0);
    // if the ball is close enough
    // squat next to the hole
   /* if (val < 0.3) {
        mGaitManager->stop();
        wait(300); 
        while(1){
        mMotionManager->playPage(15); // squat down
        wait(10000);        
      }
     }
    */
    return false; 
}

void VisualTracking::findHole()
{
 // First step to update sensors values
    myStep();
    
    // set eye led to green
    //mEyeLED->set(0x00FF00);
    
    mMotionManager->playPage(9); // init position
    wait(200);
    
    // play the motion preparing the robot to walk
    mGaitManager->start();
    mGaitManager->step(mTimeStep);
    
    double px = 0.0;
    double py = 0.0;
    int fup = 0;
    int fdown = 0;
    const double acc_tolerance = 80.0;
    const double acc_step = 20;
    
    double headPosition = 0;
    bool holeFound = false;
    Point p = applyFilter();
    Point hole; 
    
    static int width  = mCamera->getWidth();
    static int height = mCamera->getHeight();
          
          
    while (true) {
    double x, y, neckPosition;         
         
         if(p.x !=0 && p.y !=0)
         {
             holeFound = true;
             hole.x = p.x;
             hole.y = p.y; 
         }
         
         if(checkIfFallen(fup, fdown, acc_tolerance, acc_step))
         {
         }
        
         // if the hole is in the field of view, go in the direction of hole
         else if (holeFound) {
          x = (double)hole.x ;
          y = (double)hole.y;
          
          walkTowardsHole(x, y, px, py, neckPosition, headPosition);
         }
         
         // the ball is not in the field of view,
         // search it by turning round and moving vertically the head
         else {
           lookAround(headPosition, maxMotorPositions[19] -0.3, maxMotorPositions[19], -0.05);
           p = applyFilter();
         }
         
         // step
         myStep();
    }
}







//******************
// MAIN METHOD
//************************


// function containing the main feedback loop
void VisualTracking::run() {
    
    // First step to update sensors values
    myStep();
    
    // set eye led to green
    //mEyeLED->set(0x00FF00);
    
    mMotionManager->playPage(9); // init position
    wait(200);
    
    // play the motion preparing the robot to walk
    mGaitManager->start();
    mGaitManager->step(mTimeStep);
    
    // main loop
    double px = 0.0;
    double py = 0.0;
    int fup = 0;
    int fdown = 0;
    const double acc_tolerance = 80.0;
    const double acc_step = 20;
    
    double headPosition = 0;
    
   /* while (true) {
        double x, y, neckPosition;
        bool ballInFieldOfView = getBallCenter(x, y);
        if(checkIfFallen(fup, fdown, acc_tolerance, acc_step)){}
        // if the ball is in the field of view,
        // go in the direction of the ball and squat next to it
        else if (ballInFieldOfView) {
            if(walkTowardsBall(x, y, px, py, neckPosition, headPosition))
            {
              findHole();
            }
        }
        // the ball is not in the field of view,
        // search it by turning round and moving vertically the head
        else {
            lookAround(headPosition, minMotorPositions[19], minMotorPositions[19] +0.5, -0.22);
        }
        // step
        myStep();
    }*/
    
    findHole();
}
