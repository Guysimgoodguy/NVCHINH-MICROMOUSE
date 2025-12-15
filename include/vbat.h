#ifndef VBAT_H
#define VBAT_H

#include <Arduino.h>


extern volatile float g_battery_scale;
extern volatile float g_battery_voltage;
extern volatile float g_vbat;


void update_battery_voltage();
double get_g_battery_scale();

#endif