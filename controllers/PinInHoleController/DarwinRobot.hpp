#ifndef DARWINROBOT_HPP
#define DARWINROBOT_HPP

#include <webots/Robot.hpp>
#include "Target.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#define NMOTORS 20

namespace managers {
    class DARwInOPMotionManager;
    class DARwInOPGaitManager;
    class DARwInOPVisionManager;
}

namespace webots {
    class Motor;
    class LED;
    class Camera;
    class Display;
    class PositionSensor;
    class Accelerometer;
    class Gyro;
    class Speaker;
};

using namespace cv;



class DarwinRobot : public webots::Robot {
public:
    DarwinRobot();
    ~DarwinRobot();
    
    // set robot back up to initial state.
    void setup();
    
    // search around for the specified target
    bool findTarget(Target* t);
    
    // walk towards specified target
    bool walkToTarget(Target* t);
    
    // get camera properties and image
    int getCameraWidth();
    int getCameraHeight();
    const unsigned char* getImage();
    
    // convert image to a gray opencv Mat
    Mat getMat();
    
    // get Display window
    webots::Display*  getDisplay();
    
    // get robot managers
    managers::DARwInOPVisionManager*  getVisionManager();
    managers::DARwInOPGaitManager*  getGaitManager();
    managers::DARwInOPMotionManager*  getMotionManager();
    
    // get access to motors
    webots::Motor* getMotors(int i);

    // wait for a specified number of miliseconds
    void wait(int ms);
 
    // get max and min values of specified limp positions
    double getMaxLimbPos(int i);
    double getMinLimbPos(int i);
    
    
private:
    
    // axis and axis increments
    double px, py , yAxis, xAxis;
    
    // timestep to increment robot
    int  mTimeStep;
    
    // check if robot is face up or down 
    int faceUp;
    int faceDown;
    // accelerometer step and tolerance
    double acc_tolerance;
    double acc_step;
    
    // create variables to hold webot properties
    webots::Motor           *mMotors[NMOTORS];
    webots::Accelerometer   *mAccelerometer;
    webots::Camera          *mCamera;
    webots::PositionSensor  *mPositionSensors[NMOTORS];
    webots::Display         *mDisplay;
    
    // hold minimum and maximum motot position
    double minMotorPositions[NMOTORS];
    double maxMotorPositions[NMOTORS];
    
    // create robot managers
    managers::DARwInOPMotionManager *mMotionManager;
    managers::DARwInOPGaitManager   *mGaitManager;
    managers::DARwInOPVisionManager *mVisionManager;
    
    // check to see if the robot has fallen over
    void checkIfFallen(int &fup, int &fdown, const double acc_tolerance, const double acc_step);
    
    // increase timestep
    void incrementTimestep();
    
    // if value is out of range, clamp it to min or max value
    double clamp(double value, double min , double max);
    
    // store the robot motor names
    const char *motorNames[NMOTORS] = {
        "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
        "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
        "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
        "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
        "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
    };
    
};

#endif