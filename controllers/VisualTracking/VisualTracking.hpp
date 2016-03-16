
#ifndef VISUALTRACKING_HPP
#define VISUALTRACKING_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

namespace managers {
    class DARwInOPVisionManager;
    class DARwInOPMotionManager;
    class DARwInOPGaitManager;
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

class VisualTracking : public webots::Robot {
public:
    VisualTracking();
    virtual                         ~VisualTracking();
    void                             run();

    
private:
    int                              mTimeStep;
    void                            lookAround(double headPosition, double min, double max);
    int                             walkTowardsItem(double &x, double &y, double &px, double &py, double &neckPosition, double &headPosition);
    bool                            checkIfFallen(int fup, int fdown, const double acc_tolerance, const double acc_step);
    bool                            getBallCenter(double &x, double &y);
    bool                            walkTowardsBall(double &x, double &y, double &px, double &py, double &neckPosition, double &headPosition);
    Point                           applyFilter();
    bool                            walkTowardsHole(double &x, double &y, double &px, double &py, double &neckPosition, double &headPosition);
    void                            findHole();




    void                             myStep();
    void                             wait(int ms);
    
    webots::Motor                   *mMotors[NMOTORS];
    webots::LED                     *mEyeLED;
    webots::LED                     *mHeadLED;
    webots::Camera                  *mCamera;
    webots::PositionSensor *mPositionSensors[NMOTORS];
    webots::Display                 *mDisplay;
    webots::Accelerometer           *mAccelerometer;
    
    managers::DARwInOPVisionManager *mVisionManager;
    managers::DARwInOPMotionManager *mMotionManager;
    managers::DARwInOPGaitManager   *mGaitManager;
};

#endif
