

#ifndef MOUSE_H
#define MOUSE_H
#include "Arduino.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "reporting.h"
#include "sensors.h"
#include "EEPROM.h"

class Mouse;
extern Mouse mouse;

class Mouse
{
public:
  enum State
  {
    FRESH_START,
    SEARCHING,
    INPLACE_RUN,
    SMOOTH_RUN,
    FINISHED
  };

  enum TurnType
  {
    SS90EL = 0,
    SS90ER = 1,
    SS90L = 2,
    SS90R = 3,
  };

  Mouse()
  {
    init();
  }

  void init()
  {
    m_handStart = false;
    sensors.set_steering_mode(STEERING_OFF);
    m_location = Location(0, 0);
    m_heading = NORTH;
  }

  void set_heading(Heading new_heading)
  {
    m_heading = new_heading;
  }

  void turn_IP180()
  {
    static int direction = 1;
    direction *= -1;
    motion.spin_turn(direction * 180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  void turn_IP90R()
  {
    motion.spin_turn(-90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  void turn_IP90L()
  {
    motion.spin_turn(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
  }

  void turn_smooth(int turn_id)
  {
    sensors.set_steering_mode(STEERING_OFF);
    motion.set_target_velocity(SEARCH_TURN_SPEED);
    TurnParameters params = turn_params[turn_id];

    float trigger = params.trigger;

    bool triggered_by_sensor = false;
    float turn_point = FULL_CELL + params.entry_offset;

    while (motion.position() < turn_point)
    {
      sensors.update(0);
      if (sensors.get_front_sum() < trigger)
      {
        motion.set_target_velocity(motion.velocity());
        triggered_by_sensor = true;
        break;
      }
    }
    char note = triggered_by_sensor ? 's' : 'd';
    char dir = (turn_id & 1) ? 'R' : 'L';
    BTSerial.println(note);

    motion.turn(params.angle, params.omega, 0, params.alpha);
    motion.move(params.exit_offset, motion.velocity(), SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_position(SENSING_POSITION);
  }

  void stop_at_center()
  {
    bool has_wall = sensors.see_front_wall;
    sensors.set_steering_mode(STEERING_OFF);
    float remaining = (FULL_CELL + HALF_CELL) - motion.position();

    motion.start_move(remaining, motion.velocity(), 30, motion.acceleration());
    if (has_wall)
    {
      while (sensors.get_front_sum() > FRONT_REFERENCE)
      {
        sensors.update(0);
      }
    }
    else
    {
      while (not motion.move_finished())
      {
        sensors.update(0);
      };
    }

    motion.reset_drive_system();
  }

  void move_ahead()
  {
    motion.adjust_forward_position(-FULL_CELL);
    motion.wait_until_position(SENSING_POSITION);
  }

  void turn_left()
  {
    turn_smooth(SS90EL);
    m_heading = left_from(m_heading);
  }

  void turn_right()
  {
    turn_smooth(SS90ER);
    m_heading = right_from(m_heading);
  }

  void turn_back()
  {
    stop_at_center();
    turn_IP180();
    motion.move(-BACK_WALL_TO_CENTER - 30, 100, 100, SEARCH_ACCELERATION / 2);

    motion.set_position(HALF_CELL - BACK_WALL_TO_CENTER);
    float distance = SENSING_POSITION - HALF_CELL + BACK_WALL_TO_CENTER;
    motion.move(distance, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_position(SENSING_POSITION);
    m_heading = behind_from(m_heading);
  }

  void follow_to(Location target)
  {
    BTSerial.println(F("Follow TO"));
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    maze.initialise();
    sensors.wait_for_user_start();
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    digitalToggle(PC13);
    motion.set_position(HALF_CELL);
    BTSerial.println(F("Off we go..."));
    motion.wait_until_position(SENSING_POSITION);

    while (m_location != target)
    {
      sensors.set_steering_mode(STEER_NORMAL);
      m_location = m_location.neighbour(m_heading);

      sensors.update(1);
      update_map();
      char action = '#';
      if (m_location != target)
      {
        if (!sensors.see_left_wall)
        {

          turn_left();
          action = 'L';
        }
        else if (!sensors.see_front_wall)
        {
          move_ahead();
          action = 'F';
        }
        else if (!sensors.see_right_wall)
        {

          turn_right();
          action = 'R';
        }
        else
        {

          turn_back();
          action = 'B';
        }
      }
    }

    stop_at_center();
    delay(250);
    sensors.disable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
  }

  void run(int mm)
  {
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    maze.initialise();
    sensors.wait_for_user_start();
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEER_NORMAL);
    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);
    motion.move(mm, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    delay(250);
    sensors.disable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    motion.disable_drive();
  }

  void search_to(Location target)
  {
    maze.flood(target);

    delay(200);
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    if (not m_handStart)
    {

      motion.move(-BACK_WALL_TO_CENTER - 30, 100, 0, SEARCH_ACCELERATION / 2);
    }
    motion.set_position(HALF_CELL - BACK_WALL_TO_CENTER);
    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    motion.set_position(HALF_CELL);

    motion.wait_until_position(SENSING_POSITION);
    BTSerial.println("ok");

    while (m_location != target)
    {

      sensors.set_steering_mode(STEER_NORMAL);
      m_location = m_location.neighbour(m_heading);
      sensors.update(1);
      BTSerial.print(sensors.lss.value);
      BTSerial.print(" ");
      BTSerial.print(sensors.ff.value);
      BTSerial.print(" ");
      BTSerial.println(sensors.rss.value);
      digitalToggle(PC13);

      update_map();
      maze.flood(target);
      unsigned char newHeading = maze.heading_to_smallest(m_location, m_heading);
      unsigned char hdgChange = (newHeading - m_heading) & 0x3;
      if (m_location != target)
      {
        switch (hdgChange)
        {

        case AHEAD:
          move_ahead();
          break;
        case RIGHT:

          turn_right();
          break;
        case BACK:
          turn_back();
          break;
        case LEFT:

          turn_left();

          break;
        }
      }
    }
    BTSerial.println("VE DICH");
    reporter.print_maze(PLAIN);

    stop_at_center();
    sensors.disable();
    char path[512];
    make_path(START, path);
    expand_path(path);
    motion.stop();
    motion.disable_drive();
    motion.reset_drive_system();

    delay(250);
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
  }

  bool getRandomBool()
  {
    return rand() % 2 == 0;
  }

  uint8_t randomHeading()
  {
    uint8_t turnDirection;
    bool leftWall = sensors.see_left_wall;
    bool rightWall = sensors.see_right_wall;
    bool frontWall = sensors.see_front_wall;

    if (leftWall && rightWall && frontWall)
    {
      turnDirection = BACK;
    }
    else if (leftWall && rightWall)
    {
      turnDirection = AHEAD;
    }
    else if (rightWall && frontWall)
    {
      turnDirection = LEFT;
    }
    else if (leftWall && frontWall)
    {
      turnDirection = RIGHT;
    }
    else if (leftWall)
    {
      if (getRandomBool())
      {
        turnDirection = RIGHT;
      }
      else
      {
        turnDirection = AHEAD;
      }
    }
    else if (rightWall)
    {
      if (getRandomBool())
      {
        turnDirection = LEFT;
      }
      else
      {
        turnDirection = AHEAD;
      }
    }
    else
    {
      if (getRandomBool())
      {
        turnDirection = LEFT;
      }
      else
      {
        turnDirection = RIGHT;
      }
    }

    return turnDirection;
  }

  void run_to(Location target)
  {
    (void)target;
  }

  void turn_to_face(Heading newHeading)
  {
    unsigned char hdgChange = (newHeading + HEADING_COUNT - m_heading) % HEADING_COUNT;
    switch (hdgChange)
    {
    case AHEAD:
      break;
    case RIGHT:
      turn_IP90R();
      break;
    case BACK:
      turn_IP180();
      break;
    case LEFT:
      turn_IP90L();
      break;
    }
    m_heading = newHeading;
  }

  void update_map()
  {
    bool leftWall = sensors.see_left_wall;
    bool frontWall = sensors.see_front_wall;
    bool rightWall = sensors.see_right_wall;
    BTSerial.print(leftWall);
    BTSerial.print(frontWall);
    BTSerial.println(rightWall);
    char w[] = "--- ";
    if (leftWall)
    {
      w[0] = 'L';
    };
    if (frontWall)
    {
      w[1] = 'F';
    };
    if (rightWall)
    {
      w[2] = 'R';
    };
    BTSerial.print(w);
    switch (m_heading)
    {
    case NORTH:
      maze.update_wall_state(m_location, NORTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, leftWall ? WALL : EXIT);
      break;
    case EAST:
      maze.update_wall_state(m_location, EAST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, leftWall ? WALL : EXIT);
      break;
    case SOUTH:
      maze.update_wall_state(m_location, SOUTH, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, WEST, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, EAST, leftWall ? WALL : EXIT);
      break;
    case WEST:
      maze.update_wall_state(m_location, WEST, frontWall ? WALL : EXIT);
      maze.update_wall_state(m_location, NORTH, rightWall ? WALL : EXIT);
      maze.update_wall_state(m_location, SOUTH, leftWall ? WALL : EXIT);
      break;
    default:

      break;
    }
  }

  void make_path(Location start, char *path)
  {
    Location now = start;
    Heading heading = NORTH;
    int pathIndex = 0;
    path[pathIndex++] = 'B';

    while (maze.cost(now) != 0)
    {
      Heading bestDir = heading;
      uint16_t minCost = 0xFFFF;

      Heading checkDirs[] = {heading, right_from(heading), left_from(heading), behind_from(heading)};

      for (int i = 0; i < 4; i++)
      {
        Heading dir = checkDirs[i];
        if (maze.is_exit(now, dir))
        {
          Location next = now.neighbour(dir);
          uint16_t cost = maze.cost(next);
          if (cost < minCost)
          {
            minCost = cost;
            bestDir = dir;
          }
        }
      }

      unsigned char hdgChange = (bestDir + HEADING_COUNT - heading) % HEADING_COUNT;

      switch (hdgChange)
      {
      case AHEAD:
        path[pathIndex++] = 'F';
        break;
      case RIGHT:
        path[pathIndex++] = 'R';
        break;
      case LEFT:
        path[pathIndex++] = 'L';
        break;
      case BACK:
        path[pathIndex++] = 'A';
        break;
      }

      heading = bestDir;
      now = now.neighbour(heading);

      if (pathIndex > 250)
        break;
    }

    path[pathIndex++] = 'S';
    path[pathIndex] = '\0';

    BTSerial.print("Path: ");
    BTSerial.println(path);
  }

  char cmd[128];

  void expand_path(char *pathString)
  {
    int pathIndex = 0;
    int commandIndex = 0;
    cmd[commandIndex++] = 'B';
    while (char c = pathString[pathIndex])
    {
      switch (c)
      {
      case 'F':
        cmd[commandIndex++] = 'H';
        cmd[commandIndex++] = 'H';
        pathIndex++;
        break;
      case 'R':
        cmd[commandIndex++] = 'R';
        cmd[commandIndex++] = 'H';
        cmd[commandIndex++] = 'H';
        pathIndex++;
        break;
      case 'L':
        cmd[commandIndex++] = 'L';
        cmd[commandIndex++] = 'H';
        cmd[commandIndex++] = 'H';
        pathIndex++;
        break;
      case 'S':
        cmd[commandIndex++] = 'S';
        pathIndex++;
        break;
      case 'B':
      case ' ':
        pathIndex++;
        break;
      default:
        break;
      }
    }
    cmd[commandIndex] = '\0';
  }
  void save_path(char *path)
  {
    int address = 0;
    EEPROM.write(address++, 0x50);
    int i = 0;
    while (path[i] != '\0' && i < 512)
    {
      EEPROM.write(address++, path[i]);
      i++;
    }
    EEPROM.write(address + i, '\0');
    BTSerial.println("Path Saved to EEPROM");
  }

  void load_path(char *path)
  {
    BTSerial.println("LOADING...");
    int address = 0;
    int i = 0;
    char c = EEPROM.read(address++);
    while (c != '\0' && i < 512)
    {
      path[i++] = c;
      c = EEPROM.read(address++);
    }
    path[i] = '\0';
  }

  int search_maze()
  {
    sensors.wait_for_user_start();
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    search_to(maze.goal());
    maze.flood(START);

    Heading best_direction = maze.heading_to_smallest(m_location, m_heading);
    turn_to_face(best_direction);
    m_handStart = false;
    search_to(START);
    turn_to_face(NORTH);
    char path[512];
    make_path(START, path);
    expand_path(path);
    motion.stop();
    motion.disable_drive();
    motion.reset_drive_system();
    delay(5000);

    return 0;
  }

  void move_forward(float distance, float top_speed, float end_speed)
  {
    forwardProfile.start(distance, top_speed, end_speed, 2000);
    while (not forwardProfile.is_finished())
    {
      sensors.update(1);
    }
  }

  void turn(float angle, float omega, float alpha)
  {
    sensors.set_steering_mode(STEERING_OFF);

    rotationProfile.reset();
    rotationProfile.start(angle, omega, 0, alpha);
    while (not rotationProfile.is_finished())
    {

      delay(2);
    }
    sensors.set_steering_mode(STEER_NORMAL);
  }

  void turnSS90L()
  {
    turn(90, SPEEDMAX_SMOOTH_TURN, 2000);
  }

  void turnSS90R()
  {
    turn(-90, SPEEDMAX_SMOOTH_TURN, 2000);
  }

  void run_smooth_turns()
  {
    delay(1000);
    sensors.enable();
    motion.reset_drive_system();
    int index = 0;
    while (cmd[index] != 'S')
    {
      if (cmd[index] == 'B')
      {
        move_forward(13, SPEEDMAX_SMOOTH_TURN, SPEEDMAX_SMOOTH_TURN);
        index++;
      }

      if (cmd[index] == 'H' && cmd[index + 1] == 'R' && cmd[index + 2] == 'H')
      {
        move_forward(2, 300, SPEEDMAX_SMOOTH_TURN);

        turnSS90R();
        move_forward(2, 300, 300);
        index += 3;
      }
      else if (cmd[index] == 'H' && cmd[index + 1] == 'L' && cmd[index + 2] == 'H')
      {
        move_forward(2, 300, SPEEDMAX_SMOOTH_TURN);

        turnSS90L();
        move_forward(2, 300, 300);
        index += 3;
      }
      else if (cmd[index] == 'H' && cmd[index + 1] == 'H')
      {

        move_forward(HALF_CELL, 300, 300);
        index++;
      }
      else if (cmd[index] == 'H' && cmd[index + 1] == 'S')
      {

        move_forward(HALF_CELL, 300, 0);
        index++;
      }
      else
      {

        break;
      }
    }
  }

  void test_SS90E()
  {

    uint8_t side = sensors.wait_for_user_start();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);

    float distance = BACK_WALL_TO_CENTER + HALF_CELL;
    motion.move(distance, SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, SEARCH_ACCELERATION);
    motion.set_position(FULL_CELL);

    if (side == RIGHT_START)
    {
      turn_smooth(SS90ER);
    }
    else
    {
      turn_smooth(SS90EL);
    }

    int sensor_left = sensors.lss.value;
    int sensor_right = sensors.rss.value;

    motion.move(2 * FULL_CELL, SEARCH_TURN_SPEED, 0, SEARCH_ACCELERATION);
    sensor_left -= sensors.lss.value;
    sensor_right -= sensors.rss.value;
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
  }

private:
  Heading m_heading;
  Location m_location;
  bool m_handStart = false;
};

#endif