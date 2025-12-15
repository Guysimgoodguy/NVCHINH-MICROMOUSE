#include "vbat.h"
#include "config.h"
#include "adc_dma.h"

volatile float g_battery_scale = 0;
volatile float g_battery_voltage = 0;
volatile float g_vbat = 0;

static float s_filtered_vbat_raw = 0;
const float BATTERY_FILTER_ALPHA = 0.05;

void update_battery_voltage()
{
  static float s_filtered_vbat = 0;
  float raw_input = (float)adcValue;

  if (s_filtered_vbat == 0)
  {
    s_filtered_vbat = raw_input;
  }

  s_filtered_vbat = (BATTERY_FILTER_ALPHA * raw_input) + ((1.0f - BATTERY_FILTER_ALPHA) * s_filtered_vbat);

  g_vbat = s_filtered_vbat;
  g_battery_voltage = (BATTERY_MULTIPLIER * g_vbat);

  if (g_battery_voltage > 0.5)
  {
    g_battery_scale = 255.0 / g_battery_voltage;
  }
}

double get_g_battery_scale()
{
  double out;
  noInterrupts();
  out = g_battery_scale;
  interrupts();
  return out;
}