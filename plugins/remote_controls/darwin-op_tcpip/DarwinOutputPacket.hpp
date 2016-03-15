/*
 * Description:  Defines a packet sending from the remote control library to the DARwIn-OP
 */

#ifndef DARWIN_OUTPUT_PACKET_HPP
#define DARWIN_OUTPUT_PACKET_HPP

#include "Packet.hpp"

class Device;

class DarwinOutputPacket : public Packet {
  public:
                 DarwinOutputPacket();
    virtual     ~DarwinOutputPacket();

    virtual void clear();

    int          answerSize() const { return mAnswerSize; }

    void         apply(int simulationTime);

    bool         isAccelerometerRequested() const { return mAccelerometerRequested; }
    bool         isGyroRequested() const { return mGyroRequested; }
    bool         isCameraRequested() const { return mCameraRequested; }
    bool         isPositionSensorRequested(int at) const { return mPositionSensorRequested[at]; }
    bool         isMotorForceFeedback(int at) const { return mMotorTorqueFeedback[at]; }

  private:
    int          mAnswerSize;

    bool         mAccelerometerRequested;
    bool         mGyroRequested;
    bool         mCameraRequested;
    bool         mPositionSensorRequested[20];
    bool         mMotorTorqueFeedback[20];
};

#endif
