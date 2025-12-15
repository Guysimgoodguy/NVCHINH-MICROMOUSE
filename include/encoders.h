#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>

int32_t encoder_left_total();
int32_t encoder_right_total();

void reset_encoders();
void setup_encoders();
void update_encoders();

float robot_fwd_increment();
float robot_rot_increment();

float robot_position();
float robot_angle();

float robot_linear_velocity();

#endif