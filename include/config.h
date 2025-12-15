#ifndef CONFIG_H
#define CONFIG_H

#include <Wire.h>
#include <VL53L0X.h>

#include <SparkFun_TB6612.h>
#include <SoftwareSerial.h>
#include <mpu.h>

#define PI 3.1415926535897932384626433832795
#define BTSerial Serial1

const int offsetA = 1;
const int offsetB = 1;

#define TCAADDR 0x70

#define X_SHUTL PA6
#define X_SHUTR PB8
#define X_SHUTF PB3

#define VBAT PA7

#define ADDRF 0x30
#define ADDRL 0x31
#define ADDRR 0x32

#define PWMA PA1
#define PWMB PA5
#define BIN1 PB14
#define BIN2 PB15
#define STBY PA4
#define AIN1 PB4
#define AIN2 PB5
#define BTN PA15

#define ENCAL PB0
#define ENCBL PB1

#define ENCAR PA3
#define ENCBR PA2

#define WALL_KP 0.1

const int enca[] = {ENCAL, ENCAR};
const int encb[] = {ENCBL, ENCBR};

const float WHEEL_DIAMETER = 40;
const float ENCODER_PULSES = 28;
const float GEAR_RATIO = 10;

const float FWD_KM = 447.80308;
const float FWD_TM = 0.53757;

const float ROT_KM = 711.48514;
const float ROT_TM = 0.29213;

const float MM_PER_COUNT = PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float RADIANS_PER_DEGREE = 2 * PI / 360.0;

const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

extern float g_mouse_radius;

extern float g_deg_per_mm_difference;

void set_mouse_radius(float radius);

const int BACK_WALL_TO_CENTER = 35;
const float MOUSE_RADIUS = 38.7619;
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));
const float ROTATION_BIAS = -0.0004;

const int REPORTING_INTERVAL = 10;

const float FULL_CELL = 160.0f;
const float HALF_CELL = FULL_CELL / 2.0;

const float BATTERY_R1 = 55400.0;
const float BATTERY_R2 = 32200.0;
const float BATTERY_DIVIDER_RATIO = BATTERY_R2 / (BATTERY_R1 + BATTERY_R2);
const float ADC_FSR = 4095.0;
const float ADC_REF_VOLTS = 3.34;

const float SPEED_FF = (1.0 / FWD_KM);
const float BIAS_FF = (373.41286 / FWD_KM);

const float BATTERY_MULTIPLIER = (ADC_REF_VOLTS / ADC_FSR / BATTERY_DIVIDER_RATIO);
const float BATTERY_BIAS = 0.5846;

const float MAX_MOTOR_VOLTS = 7.7;

const float ACC_FF = (FWD_TM / FWD_KM);
const float TOP_SPEED = (MAX_MOTOR_VOLTS - BIAS_FF) / SPEED_FF;

const float FWD_ZETA = 0.707;
const float FWD_TD = FWD_TM;

const float FWD_KP = 16 * FWD_TM / (FWD_KM * FWD_ZETA * FWD_ZETA * FWD_TD * FWD_TD);
const float FWD_KD = LOOP_FREQUENCY * (8 * FWD_TM - FWD_TD) / (FWD_KM * FWD_TD);

const float ROT_ZETA = 0.707;
const float ROT_TD = ROT_TM;

const float ROT_KP = 16 * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD);
const float ROT_KD = LOOP_FREQUENCY * (8 * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD);

const int OMEGA_SPIN_TURN = 360;
const int ALPHA_SPIN_TURN = 3600;

const int SEARCH_SPEED = 100;
const int SEARCH_ACCELERATION = 2000;
const int SEARCH_TURN_SPEED = 100;
const int SMOOTH_TURN_SPEED = 100;
const int FAST_TURN_SPEED = 600;
const int FAST_RUN_SPEED_MAX = 2500;

struct TurnParameters
{
  int speed;
  int entry_offset;
  int exit_offset;
  float angle;
  float omega;
  float alpha;
  int trigger;
};

const TurnParameters turn_params[4] = {
    {SEARCH_TURN_SPEED, 100, 10, 90.0, 280.0, 2000.0, 120},
    {SEARCH_TURN_SPEED, 100, 10, -90.0, 280.0, 2000.0, 120},
    {SEARCH_TURN_SPEED, 100, 10, 90.0, 280.0, 2000.0, 120},
    {SEARCH_TURN_SPEED, 100, 10, -90.0, 280.0, 2000.0, 120},
};

#define MPU_ADDR 0x68
#define MPU_YAW_AXIS 0

extern volatile float g_robot_angle_mpu;
extern volatile float g_robot_tilt_kalman;

const float SENSING_POSITION = 130.0;
const int FRONT_REFERENCE = 80.0;
#define SPEEDMAX_SMOOTH_TURN 300

#endif