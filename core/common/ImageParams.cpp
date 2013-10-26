#include <common/RobotInfo.h>

ImageParams::ImageParams(Camera::Type camera) {
  if(camera == Camera::TOP) {
    width = 640;
    height = 480;
  } else {
    width = 320;
    height = 240;
  }
  defaultHorizontalStepScale = 2;
  defaultVerticalStepScale = 2;

  size = width * height;
  rawSize = size * 2;
  factor = width / 160;
  origFactor = width / 640.0f;
}
