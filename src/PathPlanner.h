#ifndef __PathPlanner_h__
#define __PathPlanner_h__

#include <vector>
#include "Vehicle.h"

class PathPlanner
{
public:
  PathPlanner(Vehicle car);
  ~PathPlanner();
  void update();
  void generatePath(); 

 private:
  std::vector<double> x_path;
  std::vector<double> y_path;
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  Vehicle car;

}; 
#endif // __PathPlanner_h__
