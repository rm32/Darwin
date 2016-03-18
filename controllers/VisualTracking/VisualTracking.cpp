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
        mGaitManager->setXAmplitude(1.0);
      else // ball close, go slowly
        mGaitManager->setXAmplitude(0.5);
      mGaitManager->setAAmplitude(neckPosition + 0.65);
      mGaitManager->step(mTimeStep);
      
      // Move head
      mMotors[18]->setPosition(neckPosition);
      mMotors[19]->setPosition(headPosition);
      
      // if the ball is close enough
      // kick the ball with the right foot
      if (y > 0.4) {
        mGaitManager->stop();
        wait(300);
        mMotionManager->playPage(15); // squat down
        wait(100);
        touchBall();
        // set eye led to green
        mEyeLED->set(0x00FF00);
        return true; 
      }
    
    return false; 
}


void VisualTracking::touchBall()
{
  mMotors[4]->setPosition(-1.65); // lower arm
  mMotors[0]->setPosition(0.6);   // upper arm
  mMotors[8]->setPosition(0.7);   // right leg
  wait(10000);
  
  mMotionManager->playPage(9); // init position
  wait(200);
    
} 



//*********************************************
// METHOD FOR FILTER
//*********************************************
Mat filter(Mat &src)
{
    
    // Mat to hold the filtered image
    Mat filterSrc;

    /// Convert the image to Gray
    cvtColor(src, filterSrc, COLOR_BGRA2GRAY);
   // Canny( filterSrc, filterSrc, 100, 3*100, 3);

    //threshold(filterSrc, filterSrc, 127.0, 255.0, THRESH_BINARY);	
    // smoothes out the image - Normalized Block Filter
    blur(filterSrc, filterSrc, Size(7, 7));		
    //smoothes out the image - Gaussian Filter
   GaussianBlur(filterSrc, filterSrc, Size(9, 9), 2, 2);	
    return filterSrc;
}

int circleCount;
int radius;

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
     radius = cvRound(circles[i][2]);
     // circle center
     circle(orig, center, 3, Scalar(0, 0, 255), -1, 8, 0);
     // circle outline
     circle(orig, center, radius, Scalar(0, 0, 255), 3, 8, 0);
     }
     
    circleCount= circles.size();
     
    return center;
}


/** @function thresh_callback */
Point thresh_callback(Mat filtered)
{
RNG rng(12345);
    Mat copy = filtered; 
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges     

    /// Find contours
    findContours( copy, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minEllipse( contours.size() );
    
    
    
    if( contours[0].size() > 5 ){
     minEllipse[0] = fitEllipse( Mat(contours[0]) ); 
    }
    
    /// Draw contours + rotated rects + ellipses
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        // ellipse
        ellipse( copy, minEllipse[i], color, 2, 8 );
     }
     
     RotatedRect rect = minEllipse[0];
     Point p = rect.center;
     return p;

}

//*********************
// METHODS TO FIND HOLE
//****************


Point VisualTracking::applyFilter(bool circle)
{
    static int width  = mCamera->getWidth();
    static int height = mCamera->getHeight();
    Point p;
    
    // get camera image
    Mat img = Mat(Size(width, height), CV_8UC4);
    img.data= (uchar*) mCamera->getImage();
    
    filteredImage = filter(img);
    if(circle)
    {
       p = circleDetection(filteredImage , filteredImage) ;
    }else{
       circleDetection(filteredImage , filteredImage) ;
       Mat image = filteredImage.clone(); 
       p = thresh_callback(image);
    }

    // convert image back to RGB so it can be shown in display
    cvtColor(filteredImage, filteredImage, COLOR_GRAY2RGB);
    
    const unsigned char *image=  filteredImage.data;
    
    ImageRef *i = mDisplay->imageNew(width, height, image, WB_IMAGE_RGB);
    
    mDisplay->imagePaste(i, 0, 0);
    
    return p;
}


void VisualTracking::armInHole()
{
while(1){
  mMotors[4]->setPosition(-0.8); // lower arm
  mMotors[0]->setPosition(1);   // upper arm
  wait(10000);    
  }
} 


bool found = false; 
int countSteps = -1; 
int x = 0;
int currentRadius = 0;

bool VisualTracking::walkTowardsHole(double &f, double &g, double &px, double &py, double &neckPosition, double &headPosition)
{

Point p;
 
double width  = mCamera->getWidth();   
double centerWidth = width/2 +25;

  if(x == 0 && (!found))
   {
      mGaitManager->setYAmplitude(0);
      mGaitManager->setXAmplitude(0);
      mGaitManager->setAAmplitude(0);

      p = applyFilter(false);
      x = p.x;
   }
   
   if((x > centerWidth + 2) && (!found))
   {
      //moveLeft
      mGaitManager->setAAmplitude(0);
      mGaitManager->setXAmplitude(0);
      mGaitManager->setYAmplitude(1);
    }else if((x < centerWidth -2)  && (!found) && x >0)
    {
      // moveRight
      mGaitManager->setXAmplitude(0);
      mGaitManager->setAAmplitude(0);
      mGaitManager->setYAmplitude(-1);     
    }else if(x ==0 && (!found))
     {
      mGaitManager->setYAmplitude(0);
      mGaitManager->setXAmplitude(0);
      mGaitManager->setAAmplitude(0);
     }else{
      
     if(circleCount > 0){ 
      if(radius > 39 && (radius > currentRadius))
      {
      cout << radius<< endl;
        currentRadius = radius; 
        countSteps = 0;
      }
     }
      
     cout <<  countSteps << endl;
         
     if(countSteps > -1)
     {
       countSteps++;
     }
    
     if(countSteps == 290)
     {
       mGaitManager->stop();
       armInHole();
     } 
      
      
      mGaitManager->setYAmplitude(0);
      mGaitManager->setAAmplitude(0);
      mGaitManager->setXAmplitude(1.0);      
      found = true; 
    }  

  
    p = applyFilter(true); 
    x =  p.x;  
    mGaitManager->step(mTimeStep);

    return true; 
}

void VisualTracking::findHole()
{
 // First step to update sensors values
    myStep();
    
    // set eye led to green
    mEyeLED->set(0x00FF00);
    
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
    Point p = applyFilter(true);
    Point hole; 
          
    while (true) {
    double x, y, neckPosition;         
         
         if(!holeFound && p.x !=0 && p.y !=0)
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
          x = 0; 
          y=0; 
          holeFound =  walkTowardsHole(x,y,px, py, neckPosition, headPosition);
         }
         
         // the ball is not in the field of view,
         // search it by turning round and moving vertically the head
         else {
           lookAround(headPosition, maxMotorPositions[19] -0.3, maxMotorPositions[19], -0.2);
           p = applyFilter(true);
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
    
    while (true) {
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
            lookAround(headPosition, minMotorPositions[19], minMotorPositions[19] +0.5, -0.4);
        }
        // step
        myStep();
    } 

    }

// right shoulde:        mMotors[0]->setPosition(0.8);
// left shoulder: mMotors[1]->setPosition(-1.2);
// right shoulder out: mMotors[2]->setPosition(1.2);
// left shoulder angle :   mMotors[3]->setPosition(0.8);
 // right arm: mMotors[4]->setPosition(0.8);;
// left arm down :  mMotors[5]->setPosition(0.8);
// hip twist:     mMotors[6]->setPosition(0.8);
// left knee:        mMotors[7]->setPosition(0.8);
// right knee:        mMotors[8]->setPosition(0.8);


