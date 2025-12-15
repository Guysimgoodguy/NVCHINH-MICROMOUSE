#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include <config.h>

const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);

static volatile float s_robot_position;
static volatile float s_robot_v;
static volatile float instantaneous_velocity;
static volatile float s_robot_angle;

static float s_robot_fwd_increment = 0;
static float s_robot_rot_increment = 0;

int encoder_left_counter;
int encoder_right_counter;

static volatile int32_t s_left_total;
static volatile int32_t s_right_total;

static volatile int left_delta;
static volatile int right_delta;

static void _handleA1()
{
    bool A = digitalRead(ENCAL);
    bool B = digitalRead(ENCBL);
    if (A == B)
        encoder_left_counter++;
    else
        encoder_left_counter--;
}
static void _handleB1()
{
    bool A = digitalRead(ENCAL);
    bool B = digitalRead(ENCBL);
    if (A != B)
        encoder_left_counter++;
    else
        encoder_left_counter--;
}
static void _handleA2()
{
    bool A = digitalRead(ENCAR);
    bool B = digitalRead(ENCBR);
    if (A == B)
        encoder_right_counter++;
    else
        encoder_right_counter--;
}
static void _handleB2()
{
    bool A = digitalRead(ENCAR);
    bool B = digitalRead(ENCBR);
    if (A != B)
        encoder_right_counter++;
    else
        encoder_right_counter--;
}

void setup_encoders()
{

    pinMode(ENCAL, INPUT_PULLUP);
    pinMode(ENCBL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCAL), _handleA1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCBL), _handleB1, CHANGE);
    encoder_left_counter = 0;

    pinMode(ENCAR, INPUT_PULLUP);
    pinMode(ENCBR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCAR), _handleA2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCBR), _handleB2, CHANGE);
    encoder_right_counter = 0;
}

void update_encoders()
{
    noInterrupts();
    left_delta = encoder_left_counter;
    right_delta = encoder_right_counter;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    interrupts();
    s_left_total += left_delta;
    s_right_total += right_delta;
    float left_change = left_delta * MM_PER_COUNT_LEFT;
    float right_change = right_delta * MM_PER_COUNT_RIGHT;
    s_robot_fwd_increment = 0.5 * (right_change + left_change);
    s_robot_rot_increment = (right_change - left_change) * g_deg_per_mm_difference;
    s_robot_position += s_robot_fwd_increment;
    s_robot_angle += s_robot_rot_increment;

    instantaneous_velocity = s_robot_fwd_increment / LOOP_INTERVAL;
    s_robot_v = (0.1f * s_robot_v) + (0.9f * instantaneous_velocity);
}

float robot_position()
{
    float dis;
    noInterrupts();
    dis = s_robot_position;
    interrupts();
    return dis;
}

float robot_fwd_increment()
{
    float dis;
    noInterrupts();
    dis = s_robot_fwd_increment;
    interrupts();
    return dis;
}
float robot_rot_increment()
{
    float dis;
    noInterrupts();
    dis = s_robot_rot_increment;
    interrupts();
    return dis;
}
float robot_angle()
{
    float ang;
    noInterrupts();
    ang = s_robot_angle;
    interrupts();
    return ang;
}

int32_t encoder_left_total()
{
    return s_left_total;
};

int32_t encoder_right_total()
{
    return s_right_total;
};

void reset_encoders()
{
    noInterrupts();
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    s_robot_position = 0;
    s_robot_v = 0;
    s_robot_angle = 0;
    s_left_total = 0;
    s_right_total = 0;
    left_delta = 0;
    right_delta = 0;
    interrupts();
}

float robot_linear_velocity()
{
    return s_robot_v;
}

#endif