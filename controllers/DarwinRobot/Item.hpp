
#ifndef ITEM_HPP
#define ITEM_HPP

#define NMOTORS 20

namespace managers {
    class DARwInOPMotionManager;
    class DARwInOPGaitManager;
    class DARwInOPVisionManager;
}

namespace webots {
    class Motor;
};

class Item {
public:
    Item();
    virtual ~Item() {}
    virtual void findLocation() = 0;
    virtual bool checkStop(double x, double y) =0;
    virtual void interactWithTarget() =0;
    double getX() const;
    double getY() const;
    bool checkIfFound() const;
    double getMaxPos(int i);
    double getMinPos(int i);
protected:
    bool isFound;
    double x;
    double y;
    double imageWidth;
    double imageHeight;
    const unsigned char *image;
    int holeLost;
    double maxPos[NMOTORS];
    double minPos[NMOTORS];
    managers::DARwInOPGaitManager* gait;
    managers::DARwInOPVisionManager* vision;
    managers::DARwInOPMotionManager* motion;
    webots::Motor *mMotors[NMOTORS];
};

#endif