/*
 * Description:  Abstraction of a Webots sensor
 */

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "Device.hpp"

class Sensor : public Device {
  public:
    // Device Manager is responsible to create/destroy devices
             Sensor(WbDeviceTag tag, int index) :
               Device(tag, index),
               mRate(0),
               mLastRefreshTime(0),
               mSensorRequested(false) {}
    virtual ~Sensor() {}

    int      lastRefreshTime() const { return mLastRefreshTime; }
    int      rate() const { return mRate; }
    bool     isEnabled() const { return mRate > 0; }

    void     setRate(int rate) { mRate = rate; }
    void     setLastRefreshTime(int lastRefreshTime) { mLastRefreshTime = lastRefreshTime; }

    bool     isSensorRequested() const { return mSensorRequested; }
    void     resetSensorRequested() { mSensorRequested = false; }
    void     setSensorRequested() { mSensorRequested = true; }

  private:
    int      mRate; // ms
    int      mLastRefreshTime; // ms
    bool     mSensorRequested;
};

#endif
