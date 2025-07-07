#ifndef ROBOTIMU_H
#define ROBOTIMU_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <MadgwickAHRS.h> 

class RobotIMU {
public:
  RobotIMU(uint8_t sdaPin, uint8_t sclPin);
  void begin();
 
  void beginMag();
  void update(); // add back the no-argument update for compatibility
  void update(float dt); // update both filters with new data

  float getAccelX();
  float getAccelY();
  float getAccelZ();

  float getGyroX();
  float getGyroY();
  float getGyroZ();

  float getPitch();  // degrees
  float getRoll();   // degrees
  float getYaw();    // degrees

  void setAlpha(float a); // set complementary filter constant
  void setMadgwickSampleRate(float rate); // set Madgwick filter sample rate
  float getFilteredPitch(); // for balancing
  float getFilteredRoll();
  float getMadgwickPitch(); 
  float getMadgwickRoll();
  float getMadgwickYaw();
  float getBalanceAngle(); // alias for getFilteredPitch

private:
  TwoWire _wire;
  MPU9250_asukiaaa _mpu;
  uint8_t _sdaPin;
  uint8_t _sclPin;
  float _filteredPitch = 0;
  float _filteredRoll = 0;
  float _gyroYBias = 0;
  float alpha = 0.5; // complementary filter constant (was const, now mutable)
  unsigned long _lastUpdate = 0;
  Madgwick _filter;
};

#endif