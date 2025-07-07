#include "RobotIMU.h"
#include <math.h>

RobotIMU::RobotIMU(uint8_t sdaPin, uint8_t sclPin)
  : _wire(1), _sdaPin(sdaPin), _sclPin(sclPin) {}

void RobotIMU::begin() {
  _wire.begin(_sdaPin, _sclPin);
  _mpu.setWire(&_wire);

  _mpu.beginAccel();
  _mpu.beginGyro();

  // Gyro Y bias calibration
  float sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; ++i) {
    _mpu.gyroUpdate();
    sum += _mpu.gyroY();
    delay(2);
  }
  _gyroYBias = sum / samples;

  // Initialize filtered pitch to current accel pitch
  _mpu.accelUpdate();
  float accelPitch = atan2(_mpu.accelY(), sqrt(_mpu.accelX() * _mpu.accelX() + _mpu.accelZ() * _mpu.accelZ())) * 180.0 / PI;
  _filteredPitch = accelPitch;


}

void RobotIMU::update() {
  _mpu.accelUpdate();
  _mpu.gyroUpdate();
  _mpu.magUpdate();
}

float RobotIMU::getAccelX() {
  return _mpu.accelX();
}

float RobotIMU::getAccelY() {
  return _mpu.accelY();
}

float RobotIMU::getAccelZ() {
  return _mpu.accelZ();
}

float RobotIMU::getGyroX() {
  return _mpu.gyroX();
}

float RobotIMU::getGyroY() {
  return _mpu.gyroY();
}

float RobotIMU::getGyroZ() {
  return _mpu.gyroZ();
}

void RobotIMU::beginMag() {
  _mpu.beginMag();
}

float RobotIMU::getPitch() {
  float ax = _mpu.accelX();
  float ay = _mpu.accelY();
  float az = _mpu.accelZ();
  return atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
}

float RobotIMU::getRoll() {
  float ax = _mpu.accelX();
  float az = _mpu.accelZ();
  return atan2(-ax, az) * 180.0 / PI;
}


float RobotIMU::getYaw() {
  // Raw magnetometer values
  float mx = _mpu.magX();
  float my = _mpu.magY();

  float heading = atan2(my, mx) * 180.0 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

void RobotIMU::setAlpha(float a) {
  alpha = constrain(a, 0.0, 1.0);
}

void RobotIMU::update(float dt) {
  _mpu.accelUpdate();
  _mpu.gyroUpdate();
  _mpu.magUpdate();

  // Complementary filter
  float accelPitch = atan2(_mpu.accelY(), sqrt(_mpu.accelX() * _mpu.accelX() + _mpu.accelZ() * _mpu.accelZ())) * 180.0 / PI;
  float gyroPitchRate = _mpu.gyroY() - _gyroYBias; // Subtract bias!
  _filteredPitch = alpha * (_filteredPitch + gyroPitchRate * dt) + (1 - alpha) * accelPitch;

  // Madgwick filter
  float ax = _mpu.accelX();
  float ay = _mpu.accelY();
  float az = _mpu.accelZ();
  float gx = _mpu.gyroX() * DEG_TO_RAD;
  float gy = _mpu.gyroY() * DEG_TO_RAD;
  float gz = _mpu.gyroZ() * DEG_TO_RAD;
  float mx = _mpu.magX();
  float my = _mpu.magY();
  float mz = _mpu.magZ();
  _filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
}

void RobotIMU::setMadgwickSampleRate(float rate) {
  _filter.begin(rate);
}

float RobotIMU::getFilteredPitch() { return _filteredPitch; }
float RobotIMU::getFilteredRoll() { return _filteredRoll; }
float RobotIMU::getMadgwickPitch() { return _filter.getPitch(); }
float RobotIMU::getMadgwickRoll() { return _filter.getRoll(); }
float RobotIMU::getMadgwickYaw() { float yaw = _filter.getYaw(); if (yaw < 0) yaw += 360; return yaw; }
float RobotIMU::getBalanceAngle() { return getFilteredPitch(); }