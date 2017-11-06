#ifndef __ROAD_H__
#define __ROAD_H__

#include <iostream>
#include <vector>

class Road 
{
private:
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
 
public:
  Road();
  Road(std::string map_file);
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  std::vector<double> getFrenet(double x, double y, double theta);
  std::vector<double> getXY(double s, double d);
};

#endif // __ROAD_H__