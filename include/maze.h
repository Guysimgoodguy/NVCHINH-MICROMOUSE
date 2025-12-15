#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>
#include "queue.h"

#define START Location(0, 0)

enum WallState
{
  EXIT = 0,
  WALL = 1,
  UNKNOWN = 2,
  VIRTUAL = 3,
};

struct WallInfo
{
  WallState north : 2;
  WallState east : 2;
  WallState south : 2;
  WallState west : 2;
};

enum MazeMask
{
  MASK_OPEN = 0x01,
  MASK_CLOSED = 0x03,
};

enum Heading
{
  NORTH,
  EAST,
  SOUTH,
  WEST,
  HEADING_COUNT,
  BLOCKED = 99
};

inline Heading right_from(const Heading heading)
{
  return static_cast<Heading>((heading + 1) % HEADING_COUNT);
}

inline Heading left_from(const Heading heading)
{
  return static_cast<Heading>((heading + HEADING_COUNT - 1) % HEADING_COUNT);
}

inline Heading ahead_from(const Heading heading)
{
  return heading;
}

inline Heading behind_from(const Heading heading)
{
  return static_cast<Heading>((heading + 2) % HEADING_COUNT);
}

enum Direction
{
  AHEAD,
  RIGHT,
  BACK,
  LEFT,
  DIRECTION_COUNT
};

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define MAX_COST (MAZE_CELL_COUNT - 1)

class Location
{
public:
  uint8_t x;
  uint8_t y;

  Location() : x(0), y(0) {};
  Location(uint8_t ix, uint8_t iy) : x(ix), y(iy) {};

  bool is_in_maze()
  {
    return x < MAZE_WIDTH && y < MAZE_HEIGHT;
  }

  bool operator==(const Location &obj) const
  {
    return x == obj.x && y == obj.y;
  }

  bool operator!=(const Location &obj) const
  {
    return x != obj.x || y != obj.y;
  }

  Location north() const
  {
    return Location(x, (y + 1) % MAZE_HEIGHT);
  }

  Location east() const
  {
    return Location((x + 1) % MAZE_WIDTH, y);
  }

  Location south() const
  {
    return Location(x, (y + MAZE_HEIGHT - 1) % MAZE_HEIGHT);
  }

  Location west() const
  {
    return Location((x + MAZE_WIDTH - 1) % MAZE_WIDTH, y);
  }

  Location neighbour(const Heading heading) const
  {
    switch (heading)
    {
    case NORTH:
      return north();
      break;
    case EAST:
      return east();
      break;
    case SOUTH:
      return south();
      break;
    case WEST:
      return west();
      break;
    default:
      return *this;
      break;
    }
  }
};

class Maze
{
public:
  Maze()
  {
  }

  Location goal() const
  {
    return m_goal;
  }

  void set_goal(const Location goal)
  {
    m_goal = goal;
  }

  WallInfo walls(const Location cell) const
  {
    return m_walls[cell.x][cell.y];
  }

  bool has_unknown_walls(const Location cell) const
  {
    WallInfo walls_here = m_walls[cell.x][cell.y];
    if (walls_here.north == UNKNOWN || walls_here.east == UNKNOWN || walls_here.south == UNKNOWN || walls_here.west == UNKNOWN)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  int wall_count(const Location cell) const
  {
    int count = 4;
    if (is_exit(cell, NORTH))
    {
      count -= 1;
    }

    if (is_exit(cell, EAST))
    {
      count -= 1;
    }
    if (is_exit(cell, SOUTH))
    {
      count -= 1;
    }
    if (is_exit(cell, WEST))
    {
      count -= 1;
    }
    return count;
  }

  bool cell_is_visited(const Location cell) const
  {
    return not has_unknown_walls(cell);
  }

  bool is_exit(const Location cell, const Heading heading) const
  {
    bool result = false;
    WallInfo walls = m_walls[cell.x][cell.y];
    switch (heading)
    {
    case NORTH:
      result = (walls.north & m_mask) == EXIT;
      break;
    case EAST:
      result = (walls.east & m_mask) == EXIT;
      break;
    case SOUTH:
      result = (walls.south & m_mask) == EXIT;
      break;
    case WEST:
      result = (walls.west & m_mask) == EXIT;
      break;
    default:
      result = false;
      break;
    }
    return result;
  }

  void update_wall_state(const Location cell, const Heading heading, const WallState state)
  {
    switch (heading)
    {
    case NORTH:
      if ((m_walls[cell.x][cell.y].north & UNKNOWN) != UNKNOWN)
      {
        return;
      }
      break;
    case EAST:
      if ((m_walls[cell.x][cell.y].east & UNKNOWN) != UNKNOWN)
      {
        return;
      }
      break;
    case WEST:
      if ((m_walls[cell.x][cell.y].west & UNKNOWN) != UNKNOWN)
      {
        return;
      }
      break;
    case SOUTH:
      if ((m_walls[cell.x][cell.y].south & UNKNOWN) != UNKNOWN)
      {
        return;
      }
      break;
    default:

      break;
    }
    set_wall_state(cell, heading, state);
  }

  void initialise()
  {
    for (int x = 0; x < MAZE_WIDTH; x++)
    {
      for (int y = 0; y < MAZE_HEIGHT; y++)
      {
        m_walls[x][y].north = UNKNOWN;
        m_walls[x][y].east = UNKNOWN;
        m_walls[x][y].south = UNKNOWN;
        m_walls[x][y].west = UNKNOWN;
      }
    }
    for (int x = 0; x < MAZE_WIDTH; x++)
    {
      m_walls[x][0].south = WALL;
      m_walls[x][MAZE_HEIGHT - 1].north = WALL;
    }
    for (int y = 0; y < MAZE_HEIGHT; y++)
    {
      m_walls[0][y].west = WALL;
      m_walls[MAZE_WIDTH - 1][y].east = WALL;
    }
    set_wall_state(START, EAST, WALL);
    set_wall_state(START, NORTH, EXIT);

    set_mask(MASK_OPEN);
    flood(goal());
  }

  void set_mask(const MazeMask mask)
  {
    m_mask = mask;
  }

  MazeMask get_mask() const
  {
    return m_mask;
  }

  uint16_t neighbour_cost(const Location cell, const Heading heading) const
  {
    if (not is_exit(cell, heading))
    {
      return MAX_COST;
    }
    Location next_cell = cell.neighbour(heading);
    return m_cost[next_cell.x][next_cell.y];
  }

  uint16_t cost(const Location cell) const
  {
    return m_cost[cell.x][cell.y];
  }

  void flood(const Location target)
  {
    for (int x = 0; x < MAZE_WIDTH; x++)
    {
      for (int y = 0; y < MAZE_HEIGHT; y++)
      {
        m_cost[x][y] = (uint8_t)MAX_COST;
      }
    }

    Queue<Location, MAZE_CELL_COUNT / 4> queue;
    m_cost[target.x][target.y] = 0;
    queue.add(target);
    while (queue.size() > 0)
    {
      Location here = queue.head();
      uint16_t newCost = m_cost[here.x][here.y] + 1;

      for (int h = NORTH; h < HEADING_COUNT; h++)
      {
        Heading heading = static_cast<Heading>(h);
        if (is_exit(here, heading))
        {
          Location nextCell = here.neighbour(heading);
          if (m_cost[nextCell.x][nextCell.y] > newCost)
          {
            m_cost[nextCell.x][nextCell.y] = newCost;
            queue.add(nextCell);
          }
        }
      }
    }
  }

  Heading heading_to_smallest(const Location cell, const Heading start_heading) const
  {
    Heading next_heading = start_heading;
    Heading best_heading = BLOCKED;
    uint16_t best_cost = cost(cell);
    uint16_t cost;
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost)
    {
      best_cost = cost;
      best_heading = next_heading;
    };
    next_heading = right_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost)
    {
      best_cost = cost;
      best_heading = next_heading;
    };
    next_heading = left_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost)
    {
      best_cost = cost;
      best_heading = next_heading;
    };
    next_heading = behind_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost)
    {
      best_cost = cost;
      best_heading = next_heading;
    };
    if (best_cost == MAX_COST)
    {
      best_heading = BLOCKED;
    }
    return best_heading;
  }

private:
  void set_wall_state(const Location loc, const Heading heading, const WallState state)
  {
    switch (heading)
    {
    case NORTH:
      m_walls[loc.x][loc.y].north = state;
      m_walls[loc.north().x][loc.north().y].south = state;
      break;
    case EAST:
      m_walls[loc.x][loc.y].east = state;
      m_walls[loc.east().x][loc.east().y].west = state;
      break;
    case WEST:
      m_walls[loc.x][loc.y].west = state;
      m_walls[loc.west().x][loc.west().y].east = state;
      break;
    case SOUTH:
      m_walls[loc.x][loc.y].south = state;
      m_walls[loc.south().x][loc.south().y].north = state;
      break;
    default:

      break;
    }
  }

  MazeMask m_mask = MASK_OPEN;
  Location m_goal{7, 7};

  uint8_t m_cost[MAZE_WIDTH][MAZE_HEIGHT];
  WallInfo m_walls[MAZE_WIDTH][MAZE_HEIGHT];
};

extern Maze maze;

#endif