#ifndef DARWINROBOT_HPP
#define DARWINROBOT_HPP

#include <webots/Robot.hpp>
#include "Item.hpp"


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define NMOTORS 20

using namespace cv;

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

class DarwinRobot : public webots::Robot {
public:
    DarwinRobot();
    ~DarwinRobot();
    bool findItem(Item* o);
    bool walkToItem(Item* o);
    int getCameraWidth();
    int getCameraHeight();
    const unsigned char* getImage();
    Mat getMat();
    managers::DARwInOPVisionManager*  getVisionManager();
    managers::DARwInOPGaitManager*  getGaitManager();
    managers::DARwInOPMotionManager*  getMotionManager();
    webots::Motor* getMotors(int i);
     void wait(int ms);
    webots::Display*  getDisplay();
    bool notSet;
    void setup();
    double getMaxPos(int i);
    double getMinPos(int i);
    
    
private:
    
    double px, py , y, x;
    bool checkIfFallen(int &fup, int &fdown, const double acc_tolerance, const double acc_step);
    void updateSensorValues();
   
    double clamp(double a, double b , double c);
    
    
    int  mTimeStep;
    webots::Motor *mMotors[NMOTORS];
    webots::Accelerometer *mAccelerometer;
    webots::Camera *mCamera;
    webots::PositionSensor *mPositionSensors[NMOTORS];
    
    double minMotorPositions[NMOTORS];
    double maxMotorPositions[NMOTORS];
    webots::Display *mDisplay;
    
    
    managers::DARwInOPMotionManager *mMotionManager;
    managers::DARwInOPGaitManager   *mGaitManager;
    managers::DARwInOPVisionManager *mVisionManager;
    
        int fup;
    int fdown;
    double acc_tolerance;
    double acc_step;
    
};

#endif