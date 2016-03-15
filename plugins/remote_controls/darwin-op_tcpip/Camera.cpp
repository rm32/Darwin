#include "Camera.hpp"

#include <webots/camera.h>

#include <sstream>
#include <iostream>

using namespace std;

CameraR::CameraR(WbDeviceTag tag) :
  Sensor(tag, 0)
{
  mWidth = wb_camera_get_width(tag);
  mHeight = wb_camera_get_height(tag);
}
