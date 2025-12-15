#include "config.h"

float g_mouse_radius = MOUSE_RADIUS;
float g_deg_per_mm_difference = (180.0 / (2 * g_mouse_radius * PI));

void set_mouse_radius(float radius)
{
  g_mouse_radius = radius;

  g_deg_per_mm_difference = (180.0 / (2 * g_mouse_radius * PI));

  BTSerial.println(g_mouse_radius);
  BTSerial.println(g_deg_per_mm_difference);
}