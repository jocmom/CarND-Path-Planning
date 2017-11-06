#include "constants.h"
#include "helper.h"
#include <math.h>
#include "Vehicle.h"

using namespace std;

Vehicle::Vehicle(int id) : _id(id) {}; 

Vehicle::Vehicle(int id, double x, double y, double vel, double s, double d)  
{
}

void Vehicle::updateLane()
{
  this->_lane = (int) this->_d / LANE_WIDTH;
}

void Vehicle::updateVelocity()
{
  this->_velocity = speed(_vx, _vy);
}

void Vehicle::updateYaw()
{
  double yaw = atan2(_vx, _vy);
  this->_yaw = abs(yaw) > 0.1 ? yaw : 0.0;
}

void Vehicle::update(double x, double y, double vel, double s, double d)
{
  this->_x = x;
  this->_y = y;
  this->_velocity = vel;
  this->_s = s;
  this->_d = d;
}