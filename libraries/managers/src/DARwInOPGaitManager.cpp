#include "DARwInOPGaitManager.hpp"

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>
#include <MX28.h>
#include <Walking.h>
#include <minIni.h>

#ifdef CROSSCOMPILATION
#include <DARwInOPMotionTimerManager.hpp>
#include <MotionManager.h>
#else
#include <MotionStatus.h>
#endif

#include <iostream>
#include <cmath>
#include <cstdlib>

using namespace Robot;
using namespace managers;
using namespace webots;
using namespace std;

static const string sotorNames[DGM_NSERVOS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

DARwInOPGaitManager::DARwInOPGaitManager(webots::Robot *robot, const std::string &iniFilename) :
  mRobot(robot),
  mCorrectlyInitialized(true),
  mXAmplitude(0.0),
  mAAmplitude(0.0),
  mYAmplitude(0.0),
  mMoveAimOn(false),
  mBalanceEnable(true),
  mIsWalking(false)
{
  if (!mRobot) {
    cerr << "DARwInOPGaitManager: The robot instance is required" << endl;
    mCorrectlyInitialized = false;
    return;
  }
  mBasicTimeStep = mRobot->getBasicTimeStep();

#ifndef CROSSCOMPILATION
  for (int i = 0; i < DGM_NSERVOS; i++)
    mMotors[i] = mRobot->getMotor(sotorNames[i]);
#endif
  
  minIni ini(iniFilename.c_str());
  mWalking = Walking::GetInstance();
  mWalking->Initialize();
  mWalking->LoadINISettings(&ini);
  
#ifdef CROSSCOMPILATION
  DARwInOPMotionTimerManager::MotionTimerInit();
  MotionManager::GetInstance()->AddModule((MotionModule*)mWalking);
#endif
}

DARwInOPGaitManager::~DARwInOPGaitManager() {
}

void DARwInOPGaitManager::step(int step) {
  if (step < 8) {
    cerr << "DARwInOPGaitManager: steps of less than 8ms are not supported" << endl;
    return;
  }
#ifdef CROSSCOMPILATION
  mWalking->m_Joint.SetEnableBodyWithoutHead(true, true);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(true);
  MotionManager::GetInstance()->SetEnable(true);
#endif
  
  if (mIsWalking) {
    mWalking->X_MOVE_AMPLITUDE = mXAmplitude;
    mWalking->A_MOVE_AMPLITUDE = mAAmplitude;
    mWalking->Y_MOVE_AMPLITUDE = mYAmplitude;
    mWalking->A_MOVE_AIM_ON = mMoveAimOn;
    mWalking->BALANCE_ENABLE = mBalanceEnable;
  }

#ifndef CROSSCOMPILATION
  int numberOfStepToProcess = step / 8;

  if (mBalanceEnable && (mRobot->getGyro("Gyro")->getSamplingPeriod() <= 0)) {
    cerr << "The Gyro is not enable. DARwInOPGaitManager need the Gyro tu run the balance algorithm. The GYro  will be automatically enable."<< endl;
    mRobot->getGyro("Gyro")->enable(mBasicTimeStep);
    myStep();
  }

  for (int i = 0; i < numberOfStepToProcess; i++) {
    if (mBalanceEnable) {
      const double *gyro = mRobot->getGyro("Gyro")->getValues();
      MotionStatus::RL_GYRO = gyro[0] - 512;  // 512 = central value, skip calibration step of the MotionManager,
      MotionStatus::FB_GYRO = gyro[1] - 512;  // because the influence of the calibration is imperceptible.
    }
    mWalking->Process();
  }

  for (int i = 0; i < (DGM_NSERVOS - 2); i++)
    mMotors[i]->setPosition(valueToPosition(mWalking->m_Joint.GetValue(i+1)));
#endif
}

void DARwInOPGaitManager::stop() {
  mIsWalking = false;
  mWalking->Stop();
  while(mWalking->IsRunning())
    this->step(8);
#ifdef CROSSCOMPILATION
  // Reset Goal Position of all motors (except Head) after walking //
  for (int i = 0; i < (DGM_NSERVOS - 2); i++)
    mRobot->getMotor(sotorNames[i])->setPosition(MX28::Value2Angle(mWalking->m_Joint.GetValue(i+1))*(M_PI/180));
  
  // Disable the Joints in the Gait Manager, this allow to control them again 'manualy' //
  mWalking->m_Joint.SetEnableBodyWithoutHead(false, true);
  MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
  MotionManager::GetInstance()->SetEnable(false);
#endif
}

void DARwInOPGaitManager::start() {
  mIsWalking = true;
  mWalking->Start();
}

#ifndef CROSSCOMPILATION
double DARwInOPGaitManager::valueToPosition(unsigned short value) {
  double degree = MX28::Value2Angle(value);
  double position = degree / 180.0 * M_PI;
  return position;
}

void DARwInOPGaitManager::myStep() {
  int ret = mRobot->step(mBasicTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}
#endif
