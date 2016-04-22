
#ifndef TARGET_HPP
#define TARGET_HPP

#define NMOTORS 20

namespace managers {
    class DARwInOPMotionManager;
    class DARwInOPGaitManager;
    class DARwInOPVisionManager;
}

namespace webots {
    class Motor;
};


// Target is abstract class that interaction object can extend

class Target {
    
public:
    Target();
    virtual ~Target() {}
    
    // finds the location of the target object and updates the xAxis and yAxis
    virtual void findLocation() = 0;
    
    // sets a boolean to true when an robot walking towards the target should stop
    // given the xAxis and yAxis
    virtual bool checkStop(double x, double y) =0;
    
    // allows the robot to interact with the target in a specific way related to
    // the target.
    virtual void interactWithTarget() =0;
    
    // passes the xAxis and yAxis
    double  getXAxis() const;
    double  getYAxis() const;
    
    // is set to true is the target is found
    bool    checkIfFound() const;
    
    // sets the limb position bounds when searching for a specific target
    double  getMaxLimbPos(int i);
    double  getMinLimbPos(int i);
    
protected:
    // stores the axis and the found status
    bool isFound;
    double xAxis;
    double yAxis;
    
    // holds the image, imageWidth and imageHeight from the robot camera
    double imageWidth;
    double imageHeight;
    const unsigned char *image;
    
    // stores the max and minimum allowed position of the limbs of the robot
    double maxLimbPos[NMOTORS];
    double minLimbPos[NMOTORS];
    
    // gives the target access to the robot motors
    webots::Motor *mMotors[NMOTORS];
    
    // gives the target access to the different robot managers
    managers::DARwInOPGaitManager* gait;
    managers::DARwInOPVisionManager* vision;
    managers::DARwInOPMotionManager* motion;
};

#endif