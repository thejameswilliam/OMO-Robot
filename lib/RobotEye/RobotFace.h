#ifndef ROBOTFACE_H
#define ROBOTFACE_H

#include "RobotEye.h"

class RobotFace {
public:
  RobotFace(RobotEye* left, RobotEye* right);
  void setEmotion(const String& previousEmotion = "", const String& emotion = "", float intensity = 0.0f);
  void draw();

private:
  RobotEye* leftEye;
  RobotEye* rightEye;
};

#endif