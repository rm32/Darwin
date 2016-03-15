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

//*********************************************
// Applies several filters on the image and return a filtered Mat image
// The original image is passed as a parameter
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
   // cvtColor(src, filterSrc, COLOR_BGRA2RGB);
    
    cvtColor(src, filterSrc, COLOR_BGRA2GRAY);
    // smoothes out the image - Normalized Block Filter
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
   // blur(filterSrc, filterSrc, Size(kernel_size, kernel_size), anchor);
    
    //smoothes out the image - Gaussian Filter
    GaussianBlur(filterSrc, filterSrc, Size(kernel_size, kernel_size), 2, 2);
    return filterSrc;
}


//*********************************************
// detects if the image contains a circle and return the center point
//*********************************************
Point circleDetection(Mat &filtered, Mat &orig)
{
    vector<Vec3f> circles;
    Point center(0, 0);
    
    /// Apply the Hough Transform to find the circles
    
    HoughCircles(filtered, circles, CV_HOUGH_GRADIENT, 1, 2, 76, 60, 0, 0);
    
        
    cvtColor(filtered, filtered, COLOR_GRAY2RGB);

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


static double clamp(double value, double min, double max) {
    if (min > max) {
        assert(0);
        return value;
    }
    return value < min ? min : value > max ? max : value;
}


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


void VisualTracking::checkForBall(double headPosition)
{
    // set eye led to red
    mEyeLED->set(0xFF0000);
    
    // turn round
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(-0.22);
    mGaitManager->step(mTimeStep);
    // move the head vertically
    headPosition = clamp(0.7*sin(2.0*getTime()), minMotorPositions[19], minMotorPositions[19] +0.5);
    mMotors[19]->setPosition(headPosition);

}


void VisualTracking::walkTowardsBall(double &x, double &y, double &px, double &py, double &neckPosition, double &headPosition)
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
    mGaitManager->setAAmplitude(neckPosition +0.6);
    mGaitManager->step(mTimeStep);
    
    // Move head
    mMotors[18]->setPosition(neckPosition);
    mMotors[19]->setPosition(headPosition);
    
    // if the ball is close enough
    // squat next to the ball
    if (y > 0.5) {  //1.8
        mGaitManager->stop();
        wait(300);
        mMotionManager->playPage(15); // squat down
        while(true)
        {
            wait(500);
        }
    }
} 


void VisualTracking::findHole()
{
 static int width  = mCamera->getWidth();
 static int height = mCamera->getHeight();
    
  Mat img = Mat(Size(width, height), CV_8UC4);
  img.data= (uchar*) mCamera->getImage();
    
  
  Mat fil = filter(img);
  Point p = circleDetection(fil , fil) ;

 const unsigned char *image=  fil.data;

 ImageRef *i = mDisplay->imageNew(width, height, image, WB_IMAGE_RGB);

 mDisplay->imagePaste(i, 0, 0);
 wait(20);
  

}


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
    
    while (true) {
        double x, y, neckPosition;
        bool ballInFieldOfView = getBallCenter(x, y);
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
        }
        // if the ball is in the field of view,
        // go in the direction of the ball and squat next to it
        else if (ballInFieldOfView) {

           // walkTowardsBall(x, y, px, py, neckPosition, headPosition);
        }
        
        // the ball is not in the field of view,
        // search it by turning round and moving vertically the head
        else {
          // checkForBall(headPosition);
          while(true){
                                 findHole();
                                 }

        }
        
        // step
        myStep();
    }
}
