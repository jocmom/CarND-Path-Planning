#ifndef __PathPlanner_h__
#define __PathPlanner_h__

#include <vector>
#include "Vehicle.h"
#include "Road.h"

class PathPlanner
{
public:
  PathPlanner();
  PathPlanner(Road r);
  std::vector<double> x_path() { return _x_path; };
  std::vector<double> y_path() { return _y_path; };
  void update();
  void generatePath(); 

 private:
  std::vector<double> _x_path;
  std::vector<double> _y_path;
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  Vehicle car = Vehicle(-1);
  Road road;

}; 
#endif // __PathPlanner_h__
