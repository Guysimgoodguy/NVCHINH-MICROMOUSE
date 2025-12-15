

#ifndef REPORTER_H
#define REPORTER_H

#include <Arduino.h>
#include "encoders.h"
#include "maze.h"
#include "motion.h"
#include "motors.h"
#include "profile.h"
#include "sensors.h"

const char dir_letters[] = "FRAL";
const char hdg_letters[] = "NESW";

enum MazeView
{
  PLAIN,
  COSTS,
  DIRS
};

static Stream &printer = Serial1;

class Reporter;
extern Reporter reporter;
class Reporter
{
  uint32_t s_start_time;
  uint32_t s_report_time;
  uint32_t s_report_interval = REPORTING_INTERVAL;

public:
  void set_printer(Stream &stream)
  {
    printer = stream;
  }

  void print_hex_2(unsigned char value)
  {
    if (value < 16)
    {
      printer.print('0');
    }
    printer.print(value, HEX);
  }

  void print_justified(int32_t value, int width)
  {
    int v = value;
    int w = width;
    w--;
    if (v < 0)
    {
      w--;
    }
    while (v /= 10)
    {
      w--;
    }
    while (w > 0)
    {
      printer.write(' ');
      --w;
    }
    printer.print(value);
  }

  void report_profile_header()
  {
    printer.println(F("time robotPos robotAngle fwdPos  fwdSpeed rotpos rotSpeed fwdmVolts rotmVolts"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }

  void report_sensor_track_header()
  {
    printer.println(F(" time pos angle lfs lss rss rfs cte steer"));
    s_start_time = millis();
    s_report_time = s_start_time;
  }

  void wall_sensor_header()
  {
    printer.println(F("|           RAW            |          NORMALISED       |             |            |"));
    printer.println(F("|   lf_   ls_   rs_   rf_  |    lfs   lss   rss   rfs  |   sum diff  | front_dist |"));
  }

  void print_walls()
  {
    if (sensors.see_left_wall)
    {
      printer.print('L');
    }
    else
    {
      printer.print('-');
    }
    if (sensors.see_front_wall)
    {
      printer.print('F');
    }
    else
    {
      printer.print('-');
    }
    if (sensors.see_right_wall)
    {
      printer.print('R');
    }
    else
    {
      printer.print('-');
    }
  }

  void log_action_status(char action, char note, Location location, Heading heading)
  {
    printer.print('{');
    printer.print(action);
    printer.print(note);
    printer.print('[');
    printer.print(location.x);
    printer.print(',');
    printer.print(location.y);
    printer.print(']');
    printer.print(' ');
    if (heading < HEADING_COUNT)
    {
      printer.print(hdg_letters[heading]);
    }
    else
    {
      printer.print('!');
    }
    print_justified(sensors.get_front_sum(), 4);
    printer.print('@');
    print_justified((int)motion.position(), 4);
    printer.print(' ');

    printer.print('}');
    printer.print(' ');
  }

#define POST 'o'
#define ERR '?'
#define GAP F("   ")
#define H_WALL F("---")
#define H_EXIT F("   ")
#define H_UNKN F("···")
#define H_VIRT F("###")
#define V_WALL '|'
#define V_EXIT ' '
#define V_UNKN ':'
#define V_VIRT '#'

  void print_h_wall(uint8_t state)
  {
    if (state == EXIT)
    {
      printer.print(H_EXIT);
    }
    else if (state == WALL)
    {
      printer.print(H_WALL);
    }
    else if (state == VIRTUAL)
    {
      printer.print(H_VIRT);
    }
    else
    {
      printer.print(H_UNKN);
    }
  }
  void printNorthWalls(int y)
  {
    for (int x = 0; x < MAZE_WIDTH; x++)
    {
      printer.print(POST);
      WallInfo walls = maze.walls(Location(x, y));
      print_h_wall(walls.north & maze.get_mask());
    }
    printer.println(POST);
  }

  void printSouthWalls(int y)
  {
    for (int x = 0; x < MAZE_WIDTH; x++)
    {
      printer.print(POST);
      WallInfo walls = maze.walls(Location(x, y));
      print_h_wall(walls.south & maze.get_mask());
    }
    printer.println(POST);
  }

  void print_maze(int style = PLAIN)
  {
    const char dirChars[] = "^>v<* ";
    maze.flood(maze.goal());

    for (int y = MAZE_HEIGHT - 1; y >= 0; y--)
    {
      printNorthWalls(y);
      for (int x = 0; x < MAZE_WIDTH; x++)
      {
        Location location(x, y);
        WallInfo walls = maze.walls(location);
        uint8_t state = walls.west & maze.get_mask();
        if (state == EXIT)
        {
          printer.print(V_EXIT);
        }
        else if (state == WALL)
        {
          printer.print(V_WALL);
        }
        else if (state == VIRTUAL)
        {
          printer.print(V_VIRT);
        }
        else
        {
          printer.print(V_UNKN);
        }
        if (style == COSTS)
        {
          print_justified((int)maze.cost(location), 3);
        }
        else if (style == DIRS)
        {
          unsigned char direction = maze.heading_to_smallest(location, NORTH);
          if (location == maze.goal())
          {
            direction = DIRECTION_COUNT;
          }
          char arrow = ' ';
          if (direction != BLOCKED)
          {
            arrow = dirChars[direction];
          }
          printer.print(' ');
          printer.print(arrow);
          printer.print(' ');
        }
        else
        {
          printer.print(GAP);
        }
      }
      printer.println(V_WALL);
    }
    printSouthWalls(0);
    printer.println();
  }
};

#endif