#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "sensors.h"
#include "config.h"
#include "encoders.h"
#include "profile.h"
#include "vbat.h"
class Motors;

extern Motors motors;

extern Motor mL;
extern Motor mR;

class Motors
{
public:
  void enable_controllers()
  {
    m_controller_output_enabled = true;
  }

  void disable_controllers()
  {
    m_controller_output_enabled = false;
  }

  void reset_controllers()
  {
    m_old_left_speed = 0;
    m_old_right_speed = 0;
    m_fwd_error = 0;
    m_rot_error = 0;
    m_previous_fwd_error = 0;
    m_previous_rot_error = 0;
  }

  void stop()
  {
    set_left_motor_volts(0);
    set_right_motor_volts(0);
  }

  void begin()
  {
    analogWriteFrequency(20000);
    stop();
  }

  float position_controller()
  {

    m_fwd_error += forwardProfile.increment() - robot_fwd_increment();
    float diff = m_fwd_error - m_previous_fwd_error;
    m_previous_fwd_error = m_fwd_error;
    float output = FWD_KP * m_fwd_error + FWD_KD * diff;
    return output;
  }

  float angle_controller(float steering_adjustment)
  {
    float increment = m_omega * LOOP_INTERVAL;
    m_rot_error += increment - robot_rot_increment();
    m_rot_error += steering_adjustment * LOOP_INTERVAL;
    float diff = m_rot_error - m_previous_rot_error;
    m_previous_rot_error = m_rot_error;
    float output = ROT_KP * m_rot_error + ROT_KD * diff;
    return output;
  }

  float leftFeedForward(float speed)
  {
    float leftFF = speed * SPEED_FF;
    if (speed > 0)
    {
      leftFF += BIAS_FF;
    }
    else if (speed < 0)
    {
      leftFF -= BIAS_FF;
    }
    else
    {
    }
    float acc = (speed - m_old_left_speed) * LOOP_FREQUENCY;
    m_old_left_speed = speed;
    float accFF = ACC_FF * acc;
    leftFF += accFF;
    return leftFF;
  }

  float rightFeedForward(float speed)
  {
    float rightFF = speed * SPEED_FF;
    if (speed > 0)
    {
      rightFF += BIAS_FF;
    }
    else if (speed < 0)
    {
      rightFF -= BIAS_FF;
    }
    else
    {
    }
    float acc = (speed - m_old_right_speed) * LOOP_FREQUENCY;
    m_old_right_speed = speed;
    float accFF = ACC_FF * acc;
    rightFF += accFF;
    return rightFF;
  }

  void update_controllers(float velocity, float omega, float steering_adjustment)
  {
    m_velocity = velocity;
    m_omega = omega;
    float pos_output = position_controller();
    float rot_output = angle_controller(steering_adjustment);
    float left_output = 0;
    float right_output = 0;
    left_output += pos_output;
    right_output += pos_output;
    left_output -= rot_output;
    right_output += rot_output;

    float tangent_speed = m_omega * MOUSE_RADIUS * RADIANS_PER_DEGREE;
    float left_speed = m_velocity - tangent_speed;
    float right_speed = m_velocity + tangent_speed;
    float left_ff = leftFeedForward(left_speed);
    float right_ff = rightFeedForward(right_speed);
    if (m_feedforward_enabled)
    {
      left_output += left_ff;
      right_output += right_ff;
    }
    if (m_controller_output_enabled)
    {
      set_right_motor_volts(right_output);
      set_left_motor_volts(left_output);
    }
  }

  int pwm_compensated(float desired_voltage, float battery_voltage)
  {
    int pwm = 255 * desired_voltage / battery_voltage;
    return pwm;
  }

  void set_left_motor_volts(float volts)
  {
    volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    int motorPWM = (int)(volts * g_battery_scale);
    set_left_motor_pwm(motorPWM);
  }

  void set_right_motor_volts(float volts)
  {
    volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
    int motorPWM = (int)(volts * g_battery_scale);
    set_right_motor_pwm(motorPWM);
  }

  void set_left_motor_pwm(int pwm)
  {
    pwm = constrain(pwm, -255, 255);
    mL.drive(pwm);
  }

  void set_right_motor_pwm(int pwm)
  {
    pwm = constrain(pwm, -255, 255);
    mR.drive(pwm);
  }

  int get_fwd_millivolts()
  {
    return 1000 * (get_right_motor_volts() + get_left_motor_volts());
  }

  int get_rot_millivolts()
  {
    return 1000 * (get_right_motor_volts() - get_left_motor_volts());
  }

  float get_left_motor_volts()
  {
    float volts = 0;
    noInterrupts();
    volts = m_left_motor_volts;
    interrupts();
  }

  float get_right_motor_volts()
  {
    float volts = 0;
    noInterrupts();
    volts = m_right_motor_volts;
    interrupts();
    return volts;
  }

  void set_speeds(float velocity, float omega)
  {
    noInterrupts();
    m_velocity = velocity;
    m_omega = omega;
    interrupts();
  }

private:
  bool m_controller_output_enabled = true;
  bool m_feedforward_enabled = true;
  float m_previous_fwd_error = 0;
  float m_previous_rot_error = 0;
  float m_fwd_error = 0;
  float m_rot_error = 0;
  float m_velocity = 0;
  float m_omega = 0;
  float m_old_left_speed = 0;
  float m_old_right_speed = 0;

  float m_left_motor_volts = 0;
  float m_right_motor_volts = 0;
};

#endif