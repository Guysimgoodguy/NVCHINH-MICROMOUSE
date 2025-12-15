#include "debug.h"
#include "encoders.h"

#include "motors.h"
#include "profile.h"
#include "sensors.h"
#include <Arduino.h>

static uint32_t start_time;
static uint32_t report_time;
static uint32_t report_interval = REPORTING_INTERVAL;

void report_profile()
{
  if (millis() >= report_time)
  {
    report_time += report_interval;

    BTSerial.print(' ');
    BTSerial.print(robot_position());
    BTSerial.print(' ');
    BTSerial.print(robot_angle());
    BTSerial.print(' ');
    BTSerial.print(forwardProfile.position());
    BTSerial.print(' ');
    BTSerial.print(forwardProfile.speed());
    BTSerial.print(' ');
    BTSerial.print(rotationProfile.position());
    BTSerial.print(' ');
    BTSerial.print(rotationProfile.speed());
    BTSerial.print(' ');

    BTSerial.println();
  }
}