#pragma once
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>

//declaring constants used in the program
// cells with map data that is above this value can be considereed as occupied
const int OCCUPIED_THRESHOLD = 20;

//returns the coordinates of the given index number and a map
int getX(int index, const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
{
  return index % map->info.width;
}

int getY(int index, const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
{
  return index / map->info.width;
}

int getIndex(int x, int y, const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
{
  return map->info.width * y + x;
}

//check whether the map is within bounds or not
bool is_in_bounds(int x, int y, const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
{
  return (x >= 0 && x < (int)map->info.width &&
          y >= 0 && y < (int)map->info.height);
}

//check if the gived cell is an obstacle or not
bool is_obstacle(int x, int y, const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
{
  return ((int)map->data[getIndex(x, y, map)] > OCCUPIED_THRESHOLD);
}

////check the map resolution
double map_resolution(const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
{
  return map->info.resolution;
}

//now using the formula y=m*x+b we find the values
double get_m(double x1, double y1, double x2, double y2)
{
  return (y1 - y2) / (x1 - x2);
}

double get_b(double x1, double y1, double x2, double y2)
{
  if (x1 != x2)
    return y1 - (get_m(x1, y1, x2, y2) * x1);
  else
    return x1;
}

double get_y_intercept(double x1, double y1, double x2, double y2, double checkX)
{
  double m = get_m(x1, y1, x2, y2);
  double b = get_b(x1, y1, x2, y2);
  return m * checkX + b;
}

double get_x_intercept(double x1, double y1, double x2, double y2, double checkY)
{
  double m = get_m(x1, y1, x2, y2);
  double b = get_b(x1, y1, x2, y2);
  return (checkY - b) / m;
}