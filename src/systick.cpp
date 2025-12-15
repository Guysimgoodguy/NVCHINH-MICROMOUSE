#include <Arduino.h>
#include "HardwareTimer.h"
#include "encoders.h"
#include "mpu.h"
#include "motors.h"
#include "profile.h"
#include "sensors.h"
#include "motion.h"
#include "vbat.h"
volatile bool flag_500Hz = false;

HardwareTimer *controlTimer = nullptr;

void systickISR()
{

  update_encoders();

  motion.update();
  update_battery_voltage();
  float bal = sensors.calculate_steering_adjustment();

  motors.update_controllers(motion.velocity(), motion.omega(), bal);
}

void setup_systick()
{
  controlTimer = new HardwareTimer(TIM4);
  controlTimer->setOverflow(LOOP_FREQUENCY, HERTZ_FORMAT);
  controlTimer->attachInterrupt(systickISR);
  controlTimer->resume();
}