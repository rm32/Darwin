/*
 * Description:  Abstraction of a LED
 */

#ifndef LED_HPP
#define LED_HPP

#include "Device.hpp"

class Led : public Device {
  public:
    // Device Manager is responsible to create/destroy devices
             Led(WbDeviceTag tag, int index) :
               Device(tag, index),
               mState(0),
               mLedRequested(false) {}
    virtual ~Led() {}

    int      state() const { return mState; }
    void     setState(int state) { mState = state; }

    bool     isLedRequested() const { return mLedRequested; }
    void     resetLedRequested() { mLedRequested = false; }
    void     setLedRequested() { mLedRequested = true; }

  private:
    int      mState;
    bool     mLedRequested;
};

#endif
