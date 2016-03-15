#include "entry_points.hpp"
#include "Wrapper.hpp"

bool wbr_init(WbrInterface *ri) {
  Wrapper::init();

  ri->mandatory.wbr_start = Wrapper::start;
  ri->mandatory.wbr_stop = Wrapper::stop;
  ri->mandatory.wbr_has_failed = Wrapper::hasFailed;
  ri->mandatory.wbr_robot_step = Wrapper::robotStep;
  ri->mandatory.wbr_stop_actuators = Wrapper::stopActuators;

  ri->wbr_set_refresh_rate = Wrapper::setRefreshRate;
  ri->wbr_motor_set_torque_refresh_rate = Wrapper::setRefreshRate;
  
  ri->wbr_led_set = Wrapper::ledSet;
  
  ri->wbr_camera_set_fov = Wrapper::cameraSetFOV;
  
  ri->wbr_motor_set_position = Wrapper::motorSetPosition;
  ri->wbr_motor_set_velocity = Wrapper::motorSetVelocity;
  ri->wbr_motor_set_acceleration = Wrapper::motorSetAcceleration;
  ri->wbr_motor_set_available_torque = Wrapper::motorSetAvailableTorque;
  ri->wbr_motor_set_torque = Wrapper::motorSetTorque;
  ri->wbr_motor_set_control_pid = Wrapper::motorSetControlPID;

  return true;
}

void wbr_cleanup() {
  Wrapper::cleanup();
}
