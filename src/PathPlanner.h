#ifndef __PathPlanner_h__
#define __PathPlanner_h__

#include <vector>
#include "Vehicle.h"
#include "Road.h"
#include "json.hpp"

using json = nlohmann::json;

#define COST_COUNT 1

class PathPlanner
{
public:
  PathPlanner();
  PathPlanner(Road r);
  std::vector<double> x_path() { return _x_path; };
  std::vector<double> y_path() { return _y_path; };
  void update(json data);
  void generatePath(int lane); 
  double costAll(const Vehicle& v);
  double costCollision(const Vehicle& v);
  double costLaneShift(const int lane);

 private:
  std::vector<double> _x_path;
  std::vector<double> _y_path;
  int prev_size;
  double end_path_s;
  double end_path_d;
  std::vector<Vehicle> other_cars;
  Vehicle car = Vehicle(-1);
  Road road;
  double ref_v;
  std::vector<double> costs;
  std::vector<double> cost_weights;
}; 
#endif // __PathPlanner_h__
