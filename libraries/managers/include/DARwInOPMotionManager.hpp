// Description:   Facade between webots and the darwin-op framework
//                allowing to play the Robotis motion files

#ifndef DARWINOP_MOTION_MANAGER_HPP
#define DARWINOP_MOTION_MANAGER_HPP

#include <string>

#define DMM_NSERVOS 20

namespace webots {
  class Robot;
  class Motor;
  class PositionSensor;
}

namespace Robot {
  class Action;
}

namespace managers {
  using namespace Robot;
  class DARwInOPMotionManager {
    public:
                       DARwInOPMotionManager(webots::Robot *robot);
      virtual         ~DARwInOPMotionManager();
      bool             isCorrectlyInitialized() { return mCorrectlyInitialized; }
      void             playPage(int id, bool sync = true);
      void             step(int ms);
      bool             isMotionPlaying() { return mMotionPlaying; }

    private:
      webots::Robot   *mRobot;
      bool             mCorrectlyInitialized;
      Action          *mAction;
      int              mBasicTimeStep;
      bool             mMotionPlaying;

#ifndef CROSSCOMPILATION
      void             myStep();
      void             wait(int ms);
      void             achieveTarget(int msToAchieveTarget);
      double           valueToPosition(unsigned short value);
      void             InitMotionAsync();

      webots::Motor           *mMotors[DMM_NSERVOS];
      webots::PositionSensor  *mPositionSensors[DMM_NSERVOS];
      double                   mTargetPositions[DMM_NSERVOS];
      double                   mCurrentPositions[DMM_NSERVOS];
      int                      mRepeat;
      int                      mStepnum;
      int                      mWait;
      int                      mStepNumberToAchieveTarget;
      void                    *mPage;
#else
      static void     *MotionThread(void *param);// thread function

      pthread_t        mMotionThread;// thread structure
#endif
  };
}

#endif
