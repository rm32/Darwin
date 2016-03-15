/*
 * Description:  Defines an interface wrapping the libController with this library for the DARwIn-OP
 */

#ifndef WRAPPER_HPP
#define WRAPPER_HPP

#include <webots/types.h>

class Communication;
class Time;

class Wrapper {
  public:
    // init
    static void           init();
    static void           cleanup();

    // mandatory functions
    static bool           start(void *);
    static void           stop();
    static bool           hasFailed() { return !cSuccess; }
    static int            robotStep(int);
    static void           stopActuators();

    // redefined functions
    static void           setRefreshRate(WbDeviceTag tag, int rate);
    static void           ledSet(WbDeviceTag tag, int state);
    static void           motorSetPosition(WbDeviceTag tag, double position);
    static void           motorSetVelocity(WbDeviceTag tag, double velocity);
    static void           motorSetAcceleration(WbDeviceTag tag, double acceleration);
    static void           motorSetAvailableTorque(WbDeviceTag tag, double torque);
    static void           motorSetTorque(WbDeviceTag tag, double torque);
    static void           motorSetControlPID(WbDeviceTag tag, double p, double i, double d);

    // unimplemented required functions
    static void           cameraSetFOV(WbDeviceTag tag, double fov) {}

  private:
                          Wrapper() {}
    virtual              ~Wrapper() {}

    static Communication *cCommunication;
    static Time          *cTime;
    static bool           cSuccess;
};

#endif
