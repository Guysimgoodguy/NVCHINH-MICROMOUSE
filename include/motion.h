#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include "motors.h"
#include "sensors.h"
#include "profile.h"

class Motion
{
public:
  void reset_drive_system()
  {
    motors.stop();
    motors.disable_controllers();
    reset_encoders();
    forwardProfile.reset();
    rotationProfile.reset();
    motors.reset_controllers();
    motors.enable_controllers();
  }

  void stop()
  {
    motors.stop();
  }

  void disable_drive()
  {
    motors.disable_controllers();
  }

  float position()
  {
    return forwardProfile.position();
  }

  float velocity()
  {
    return forwardProfile.speed();
  }

  float acceleration()
  {
    return forwardProfile.acceleration();
  }

  void set_target_velocity(float velocity)
  {
    forwardProfile.set_target_speed(velocity);
  }

  float angle()
  {
    return rotationProfile.position();
  }

  float omega()
  {
    return rotationProfile.speed();
  }

  float alpha()
  {
    return rotationProfile.acceleration();
  }

  void start_move(float distance, float top_speed, float final_speed, float acceleration)
  {
    forwardProfile.start(distance, top_speed, final_speed, acceleration);
  }

  bool move_finished()
  {
    return forwardProfile.is_finished();
  }

  void move(float distance, float top_speed, float final_speed, float acceleration)
  {
    forwardProfile.move(distance, top_speed, final_speed, acceleration);
  }

  void start_turn(float distance, float top_speed, float final_speed, float acceleration)
  {
    rotationProfile.start(distance, top_speed, final_speed, acceleration);
  }

  bool turn_finished()
  {
    return rotationProfile.is_finished();
  }

  void turn(float distance, float top_speed, float final_speed, float acceleration)
  {
    rotationProfile.move(distance, top_speed, final_speed, acceleration);
  }

  void update()
  {
    forwardProfile.update();
    rotationProfile.update();
  }

  void set_position(float pos)
  {
    forwardProfile.set_position(pos);
  }

  void adjust_forward_position(float delta)
  {
    forwardProfile.adjust_position(delta);
  }

  void spin_turn(float angle, float omega, float alpha)
  {
    forwardProfile.set_target_speed(0);
    while (forwardProfile.speed() != 0)
    {
      delay(2);
    }
    rotationProfile.reset();
    rotationProfile.move(angle, omega, 0, alpha);
  };

  void stop_at(float position)
  {
    float remaining = position - forwardProfile.position();
    forwardProfile.move(remaining, forwardProfile.speed(), 0, forwardProfile.acceleration());
  }

  void stop_after(float distance)
  {
    forwardProfile.move(distance, forwardProfile.speed(), 0, forwardProfile.acceleration());
  }

  void wait_until_position(float position)
  {
    while (forwardProfile.position() < position)
    {
      sensors.update(0);
    }
  }

  void wait_until_distance(float distance)
  {
    float target = forwardProfile.position() + distance;
    wait_until_position(target);
  }
};

extern Motion motion;

#endif