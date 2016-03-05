#include <DARwInOPMotionTimerManager.hpp>

#include <webots/Robot.hpp>

#include <LinuxMotionTimer.h>
#include <MotionManager.h>

using namespace Robot;
using namespace managers;
using namespace webots;
using namespace std;

DARwInOPMotionTimerManager::DARwInOPMotionTimerManager() {
}

void DARwInOPMotionTimerManager::MotionTimerInit() {
  if (!mStarted) {
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    mStarted = true;
  }
}

DARwInOPMotionTimerManager::~DARwInOPMotionTimerManager() {
}

bool DARwInOPMotionTimerManager::mStarted = false;
