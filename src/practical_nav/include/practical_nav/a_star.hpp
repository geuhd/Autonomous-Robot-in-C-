#pragma once
#include "practical_nav/map_helpers.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cstdint>
#include <cmath>
#include <iostream>

using namespace std;

//declaration of the structure
struct cell
{
  cell() : index(-1), x(-1), y(-1), theta(-1), F(INT32_MAX), G(INT32_MAX), H(INT32_MAX), prevX(-1), prevY(-1) {}
  cell(const cell &incoming);
  int index; int x; int y; double theta;
  int F; int G; int H; int prevX; int prevY;
};

cell::cell(const cell &incoming)
{
  index = incoming.index; x = incoming.x; y = incoming.y; theta = incoming.theta;
  F = incoming.F; G = incoming.G; H = incoming.H;
  prevX = incoming.prevX; prevY = incoming.prevY;
}

extern nav_msgs::msg::OccupancyGrid::SharedPtr _map;
extern cell start;
extern cell goal;
extern bool goalActive;

//defined in path_planner.cpp (it uses the ROS publisher)
void publish_waypoint(cell nextWaypoint);

// The function calculates G
int getG(int x, int y, int currentX, int currentY, int currentG)
{
  if (is_obstacle(x, y, _map)) return INT32_MAX;
  else if (x == currentX || y == currentY) return currentG + 10;
  else return currentG + 14;
}

//H would be the abss distance from the goal
int getH(int x, int y) { return (abs(goal.x - x) + abs(goal.y - y)) * 10; }

//function to calculate the value of F
int getF(int g, int h) { if (g == INT32_MAX) return g; else return g + h; }

//check if cewll of given index is in the supplied list
bool contains(vector<cell> &list, int toCheck)
{
  for (size_t i = 0; i < list.size(); i++)
  {
    if (list[i].index == toCheck) return true;
  }
  return false;
}

// trace function follows the previous cell data we left along the way and sends the next waypoint to publish waypoint to publish to our drive controller
int trace(vector<cell> &closed)
{
  vector<cell> path;
  //closed back is the goal
  path.push_back(cell(closed.back()));
  bool pathComplete = false;
  while (pathComplete == false)
  {
    bool found = false;
    //check in the closed list for the parent cell for the last cell in the path
    for (size_t i = 0; found == false && i < closed.size(); i++)
    {
      //if found the parent cell push that to the path
      if (closed[i].x == path.back().prevX && closed[i].y == path.back().prevY)
      {
        path.push_back(cell(closed[i]));
        found = true;
      }
    }
    if (found == false) break;
    //check if the path is now complete
    if (path.back().index == start.index) pathComplete = true;
  }
  // the waypoint at pat.back is currently our start point
  //By removing it the new back() will be  our first way point
  if (path.back().index != goal.index) path.pop_back();
  if (path.back().index != path.front().index)
  {
    double deltaX = path.back().x - start.x;
    double deltaY = path.back().y - start.y;
    path.back().theta = atan2(deltaY, deltaX);
  }
  publish_waypoint(path.back());
  return path.back().index;
}

// the A* algorithm
int find_path()
{
  //create open and closed lists
  // create cell object current that will be the start cell
  //add current to the open list
  vector<cell> open;
  vector<cell> closed;

  cell current(start);
  current.G = 0;
  current.H = getH(start.x, start.y);
  current.F = current.G + current.H;
  current.index = getIndex(current.x, current.y, _map);
  open.push_back(cell(current));

  //H of 0 meants w are at goal
  while (current.H > 0)
  {
    for (int x = current.x - 1; x <= current.x + 1; x++)
    {
      for (int y = current.y - 1; y <= current.y + 1; y++)
      {
        if (is_in_bounds(x, y, _map))
        {
          //if open list is empty
          if (open.size() == 0)
          { cout << "NO PATH FOUND" << endl; goalActive = false; return -1; }

          // if the cell if already in th elist
          if (contains(open, getIndex(x, y, _map)) == true)
          {
            size_t i = 0;
            while (i < open.size() && open[i].index != getIndex(x, y, _map)) i++;
            if (i >= open.size()) continue;
            int tempG = getG(x, y, current.x, current.y, current.G);
            int tempH = getH(x, y);
            int tempF = getF(tempG, tempH);
            if (tempF < open[i].F)
            { open[i].F = tempF; open[i].G = tempG; open[i].prevX = current.x; open[i].prevY = current.y; }
          }
          //cell is not in the list then we would create it and add it
          else if (contains(closed, getIndex(x, y, _map)) == false)
          {
            cell newCell;
            newCell.x = x; newCell.y = y;
            newCell.index = getIndex(x, y, _map);
            newCell.prevX = current.x; newCell.prevY = current.y;
            newCell.G = getG(x, y, current.x, current.y, current.G);
            newCell.H = getH(x, y);
            newCell.F = getF(newCell.G, newCell.H);
            if (newCell.F == INT32_MAX) closed.push_back(cell(newCell));
            else open.push_back(newCell);
          }
        }
      }
    }

    //add current from open list and add to closed list
    closed.push_back(cell(current));
    bool found = false;
    for (size_t i = 0; found == false && i < open.size(); i++)
    {
      if (open[i].index == current.index)
      {
        open.erase(open.begin() + i); found = true;
      }
    }
    if (open.empty()){ cout << "NO PATH FOUND" << endl; goalActive = false; return -1; }

    //find cell in th eopen list with the lowest value of F
    size_t lowestF = 0;
    for (size_t i = 0; i < open.size(); i++)
    {
      if (open[i].F < open[lowestF].F) lowestF = i;
    }

    // make current equal to the lowest we have at the moment
    current.index = open[lowestF].index;
    current.x = open[lowestF].x;   current.y = open[lowestF].y;
    current.theta = open[lowestF].theta;
    current.F = open[lowestF].F;   current.G = open[lowestF].G;   current.H = open[lowestF].H;
    current.prevX = open[lowestF].prevX;   current.prevY = open[lowestF].prevY;
  }

  //we have found thegoal now and we can return the way point
  goal.prevX = closed.back().x;
  goal.prevY = closed.back().y;
  closed.push_back(cell(goal));

  return trace(closed);
}