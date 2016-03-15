#include "DarwinOutputPacket.hpp"

#include "Camera.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "TripleValuesSensor.hpp"
#include "SingleValueSensor.hpp"

#include <iostream>
#include <vector>
#include <algorithm>

#include <webots/camera.h>

using namespace std;

DarwinOutputPacket::DarwinOutputPacket() :
  Packet(50000),
  mAnswerSize(0),
  mAccelerometerRequested(false),
  mGyroRequested(false),
  mCameraRequested(false)
{ 
  for (int c = 0; c < 20; c++) {
    mPositionSensorRequested[c] = false;
    mMotorTorqueFeedback[c] = false;
  }
}

DarwinOutputPacket::~DarwinOutputPacket() {
}

void DarwinOutputPacket::clear() {
  Packet::clear();

  mAnswerSize = 0;

  mAccelerometerRequested = false;
  mGyroRequested = false;
  mCameraRequested = false;
  
  for (int c = 0; c < 20; c++) {
    mPositionSensorRequested[c] = false;
    mMotorTorqueFeedback[c] = false;
  }
}

void DarwinOutputPacket::apply(int simulationTime) {

  mAnswerSize = 1;
  append(QByteArray(1, 'W'));
  append(QByteArray(1, 0)); // the total size of the packet
  append(QByteArray(1, 0)); // will be stored here
  // ---
  // Sensors
  // ---

  // the order of the sensors should match with DarwinInputPacket::decode()

  // accelerometer management
  TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();
  if (accelerometer->isSensorRequested()) {
    mAccelerometerRequested = true;
    append(QByteArray(1, 'A'));
    mAnswerSize += 3 * 4;
  }

  // gyro management
  TripleValuesSensor *gyro = DeviceManager::instance()->gyro();
  if (gyro->isSensorRequested()) {
    mGyroRequested = true;
    append(QByteArray(1, 'G'));
    mAnswerSize += 3 * 4;
  }

  // camera management
  // it's better to put the camera at the end in case of
  // retrieval after transmission troubles
  CameraR *camera = DeviceManager::instance()->camera();
  if (camera->isSensorRequested()) {
    mCameraRequested = true;
    append(QByteArray(1, 'C'));
    mAnswerSize += 4 * 320 * 240;
  }
  
  
  // ---
  // Actuators
  // ---

  // send the led commands if required
  for (int i = 0; i < 5; i++) {
    Led *led = DeviceManager::instance()->led(i);
    if (led->isLedRequested()) {
      append(QByteArray(1, 'L'));
      append(QByteArray(1, ((led->index()) & 0xFF)));
      append(QByteArray(1, (led->state() >> 16) & 0xFF));
      append(QByteArray(1, (led->state() >> 8) & 0xFF));
      append(QByteArray(1, led->state() & 0xFF));
      led->resetLedRequested();
    }
  }
  
  // Motors management
  for (int i = 0; i < 20; i++) {
    MotorR *motor = DeviceManager::instance()->motor(i);
    if (motor->isMotorRequested()) {
      append(QByteArray(1, 'S'));
      append(QByteArray(1, ((motor->index()) & 0xFF)));
      
      // Position
      if (motor->isPositionRequested()) {
        append(QByteArray(1, 'p'));
        int value = (int)((motor->position() * 2048) / M_PI);
        appendINT(value);
        motor->resetPositionRequested();
      }
      // Velocity
      if (motor->isVelocityRequested()) {
        append(QByteArray(1, 'v'));
        int value = (int)((motor->velocity() * 30) / (0.114 * M_PI));
        appendINT(value);
        motor->resetVelocityRequested();
      }
      // Acceleration
      if (motor->isAccelerationRequested()) {
        append(QByteArray(1, 'a'));
        int value = (int)(motor->acceleration() * 100000);
        appendINT(value);
        motor->resetAccelerationRequested();
      }
      // MotorForce
      if (motor->isMotorForceRequested()) {
        append(QByteArray(1, 'm'));
        int value = (int)((motor->motorForce() * 1023) / 2.5);
        appendINT(value);
        motor->resetAvailableTorqueRequested();
      }
      // ControlPID
      if (motor->isControlPIDRequested()) {
        append(QByteArray(1, 'c'));
        int p = (int)(motor->controlP() * 1000);
        int i = (int)(motor->controlI() * 1000);
        int d = (int)(motor->controlD() * 1000);
        appendINT(p);
        appendINT(i);
        appendINT(d);
        // TODO (fabien): why doing this 1000 multiplication (and division at the other side) while
        //                simply creating an appendDouble() function would be much more elegant?
        motor->resetControlPIDRequested();
      }
      // Force
      if (motor->isForceRequested()) {
        append(QByteArray(1, 'f'));
        int value = (int)((motor->torque() * 1023) / 2.5);
        appendINT(value);
        motor->resetTorqueRequested();
      }
      motor->resetMotorRequested();
    }
  }

  for (int i = 0; i < 20; i++) {
    SingleValueSensor *positionSensor = DeviceManager::instance()->positionSensor(i);
    if (positionSensor->isSensorRequested()) {
      mPositionSensorRequested[i] = true;
      append(QByteArray(1, 'P'));
      append(QByteArray(1, ((positionSensor->index()) & 0xFF)));
      mAnswerSize += 4;
    }
  }
  
  for (int i = 0; i < 20; i++) {
    MotorR *motor = DeviceManager::instance()->motor(i);
    if (motor->isSensorRequested()) {
      mMotorTorqueFeedback[i] = true;
      append(QByteArray(1, 'F'));
      append(QByteArray(1, ((motor->index()) & 0xFF)));
      mAnswerSize += 4;
    }
  }
  // This is required to end the packet
  // even if the size is correct
  append(QByteArray(1, '\0'));
  int s = size();
  char sc[2];
  sc[0] = (unsigned char)(s%256);
  sc[1] = (unsigned char)(s/256);
  mData->replace(1,2,sc,2); // write the size of the packet
}
