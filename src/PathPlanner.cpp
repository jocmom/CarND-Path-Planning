#include <iostream>
#include <vector>
#include <math.h>
#include "PathPlanner.h"
#include "helper.h"
#include "constants.h"
#include "spline.h"

using namespace std;

PathPlanner::PathPlanner(Road r) : car(-1), road(r), ref_v(0), costs(LANE_CNT, 0)
{
}

void PathPlanner::update(json data)
{
  this->car.x(data["x"]);
  this->car.y(data["y"]);
  this->car.s(data["s"]);
  this->car.d(data["d"]);
  this->car.yaw(data["yaw"]);
  this->car.speed(data["speed"]);
  this->car.updateLane();
  int target_lane = car.lane();
  
  // Previous path data given to the Planner
  auto previous_path_x = data["previous_path_x"];
  auto previous_path_y = data["previous_path_y"];
  // Previous path's end s and d values 
  this->end_path_s = data["end_path_s"];
  this->end_path_d = data["end_path_d"];

  int prev_size = previous_path_x.size();
  // use last point from previous path to smooth new path
  if(prev_size > 0) {
    car.s(this->end_path_s);
  }
  // get all points left from the previous path not eaten by the car and
  // store them in current/next path
  _x_path.clear();
  _y_path.clear();
  for(int i = 0; i < prev_size; i++)
  {
    _x_path.push_back(previous_path_x[i]);
    _y_path.push_back(previous_path_y[i]);
  }
  
  bool too_close = false;
  // reset costs
  std::fill(costs.begin(), costs.end(), 0);
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = data["sensor_fusion"];
  this->other_cars.clear();
  for(auto sensor:sensor_fusion) {
    auto v = Vehicle(sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6]);
    double future_s = v.getFutureS(prev_size);
    v.s(future_s);
    this->other_cars.push_back(Vehicle(v));
    this->costCars(v);
    if (v.lane () == car.lane())
    {
      double distance = future_s - car.s();
      // other car in same lane and too close -> we have to slow down 
      if(distance > 0 && distance < MIN_DISTANCE)
      {
        cout << "Lane: " << car.lane() << " Distance: " << distance << endl;
        cout << "my speed: " << car.speed() << " other speed: " << v.speed() << endl;
        //cout << "my s: " << car.s() << " other s: " << future_s << endl;
        too_close = true;
        if(car.speed() > v.speed() * MPS2MPH)
        {
          this->ref_v -= MAX_V_DELTA * (1 - distance * distance / (MIN_DISTANCE * MIN_DISTANCE));
        }
      }
    }
  }
  this->costLaneShift();
  target_lane = this->bestLane();
  
  cout << "COSTS: " << costs[0] << " " << costs[1] << " " << costs[2] << endl;

  // have to slow down
  if(too_close == false) {
    this->ref_v += MAX_V_DELTA;
  }
  // clamp speed  
  if(this->ref_v > REF_V) this->ref_v = REF_V;
  if(this->ref_v < 0) this->ref_v = 0 ;

  this->generatePath(target_lane);
}

void PathPlanner::generatePath(int lane)
{
  // reference speed
  vector<double> ptsx;
  vector<double> ptsy;
  int prev_size = _x_path.size();
  // Use two points that make the tangent to current car position
  double ref_x = car.x();
  double ref_y = car.y();
  double ref_yaw = deg2rad(car.yaw());
  double ref_x_prev = car.x() - cos(ref_yaw);
  double ref_y_prev = car.y() - sin(ref_yaw);
  // Use two points that make the tangent from the last points 
  // of previous path
  if(prev_size >= 2) {
    // cout << "Got previous path with length: " << prev_size <<endl;
    ref_x = _x_path[prev_size-1];
    ref_x_prev = _x_path[prev_size-2];
    ref_y = _y_path[prev_size-1];
    ref_y_prev = _y_path[prev_size-2];
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
  
  //Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x); //apply spline
  double target_dist = sqrt((target_x * target_x) + target_y * target_y);
  double x_add_on = 0;
  
  //Spline points after previous points
  for(int i = 1; i <= PATH_POINTS_CNT - prev_size; i++)
  {
    // Use points with minimum jerk
    double N = ( target_dist / ( DELTA_T * this->ref_v /2.24));
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

void PathPlanner::costCars(const Vehicle &v) 
{
  int lane = v.lane();
  if(lane > LANE_CNT) {
    return;
  }
  costDistance(v);
  costCollision(v);
  return;
}

void PathPlanner::costDistance(const Vehicle &v)
{
  int lane = v.lane();
  double distance = v.s() - car.s();
  // cars in front of car (near)
  // if( distance >= 0 && distance < MIN_DISTANCE ) {
  //   return 0.8;
  // }
  // cars in front of car (far)
  if( distance >= 0 && distance < OPTIMAL_DISTANCE ) {
    costs[lane] += (1. - distance / OPTIMAL_DISTANCE);
  }
}

void PathPlanner::costCollision(const Vehicle &v)
{
  int lane = v.lane();
  int lane_diff = abs(lane - car.lane());
  double distance = v.s() - car.s();
  // cars behind or next to car 
  if( distance > -CAR_SIZE && distance < CAR_SIZE) {
    if(lane == car.lane()) {
      costs[lane] += 0.1;
    }
    // car is in middle lane 
    else if(car.lane() == 1) {
      costs[lane] += 1.;
    }
    // car is in outer lane
    else {
      costs[lane] += 1.;
      // and blocking car in mid lane -> increase costs
      // for other outer lane because it's blocked by mid
      if(lane == 1) {
        if(car.lane() == 0) costs[2] += 1.;
        if(car.lane() == 2) costs[0] += 1.;
        // costs[(car.lane())%LANE_CNT] += 1.;
      }
    }
  }
}

void PathPlanner::costLaneShift() 
{
  for(int lane=0; lane<LANE_CNT; ++lane) {
    if(lane == car.lane()) costs[lane] += 0.;
    else if(abs(lane - car.lane()) == 1) costs[lane] += 0.3;
    else costs[lane] += 0.7;
  }
}

int PathPlanner::bestLane()
{
  double best_cost = std::numeric_limits<double>::max();
  double target_lane = 0;
  for(int i=0; i<LANE_CNT; ++i) {
    if(costs[i] < best_cost) {
      best_cost = costs[i];
      target_lane = i;
    }
  }
  return target_lane;
}

// costSpeed, costCarsOnLaneCount, costPreferMidLane, costDoubleLaneChange