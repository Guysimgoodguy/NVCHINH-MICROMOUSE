#include "mpu.h"
#include <Wire.h>
#include "config.h"

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AngleRoll, AnglePitch;
float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
float Kalman1DOutput[] = {0, 0};
float yaw_angle = 0.0;

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
}

void setup_mpu_manual()
{
  delay(1000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(50);

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 200; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    delay(2);
  }
  RateCalibrationRoll /= 200;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void read_mpu_data(float dt)
{
  gyro_signals();
  RateRoll -= RateCalibrationRoll;

  yaw_angle += RateRoll * dt;
}

void reset_angle()
{
  yaw_angle = 0;
}

float get_yaw_angle()
{
  return yaw_angle;
}