#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

void setup_mpu_manual();
void read_mpu_data(float dt);
float get_yaw_angle();
void reset_angle();
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);

extern float KalmanAngleRoll;
extern float KalmanUncertaintyAngleRoll;
extern float RateRoll, RatePitch, RateYaw;
extern float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
extern float Kalman1DOutput[];

#endif