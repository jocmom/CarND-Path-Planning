#include <iostream>
#include <math.h>
#include "PathPlanner.h"
#include "helper.h"
#include "constants.h"
#include "spline.h"

using namespace std;

PathPlanner::PathPlanner() : car(0)
{
  road = Road();
}

PathPlanner::PathPlanner(Road r) : car(0), road(r)
{
}

void PathPlanner::generatePath()
{
  int lane = 1;
  vector<double> ptsx;
  vector<double> ptsy;
  int prev_size = previous_path_x.size();
  // Use two points that make the tangent to current car position
  const double ref_v = REF_V;
  double ref_x = car.x();
  double ref_y = car.y();
  double ref_yaw = deg2rad(car.yaw());
  double ref_x_prev = car.x() - cos(ref_yaw);
  double ref_y_prev = car.y() - sin(ref_yaw);
  // Use two points that make the tangent from the last points 
  // of previous path
  if(prev_size >= 2) {
    cout << "Got previous path with length: " << prev_size <<endl;
    ref_x = previous_path_x[prev_size-1];
    ref_x_prev = previous_path_x[prev_size-2];
    ref_y = previous_path_y[prev_size-1];
    ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
  }
  // In Frenet add evenly 30m spaced points ahead of the current position						
  vector<double> next_wp0 = road.getXY(car.s()+30, (2+4*lane));
  vector<double> next_wp1 = road.getXY(car.s()+60, (2+4*lane));
  vector<double> next_wp2 = road.getXY(car.s()+90, (2+4*lane));
  
  ptsx.push_back(ref_x_prev);
  ptsx.push_back(ref_x);
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(ref_y_prev);
  ptsy.push_back(ref_y);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  // transformation to car's reference frame
  for(int i=0; i<ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
  }
  // create spline
  tk::spline s;
  s.set_points(ptsx, ptsy);
  
  // get all points left from the previous path not eaten by the car
  for(int i = 0; i < prev_size; i++)
  {
    _x_path.push_back(previous_path_x[i]);
    _y_path.push_back(previous_path_y[i]);
  }
  //Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x); //apply spline
  double target_dist = sqrt((target_x * target_x) + target_y * target_y);
  double x_add_on = 0;
  
  //Spline points after previous points
  for(int i = 1; i <= PATH_POINTS_CNT - prev_size; i++)
  {
    // Use points with minimum jerk
    double N = ( target_dist / ( DELTA_T * REF_V /2.24));
    double x_point = x_add_on + (target_x / N);
    double y_point = s(x_point);
    
    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;
    
    // transform back to original frame
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
    x_point += ref_x;
    y_point += ref_y;
    
    // push them on current path vector			
    _x_path.push_back(x_point);
    _y_path.push_back(y_point);
  }
}