#include "Vehicle.h"
#include "constants.h"
#include <math.h>

using namespace std;

Vehicle::Vehicle(double id, double x, double y, double vx, double vy, double s, double d) : 
id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d)
{
}

void Vehicle::updateLane()
{
  this->_lane = (int) this->d / LANE_WIDTH;
}

void Vehicle::updateVelocity()
{
  this->_velocity sqrt(vx*vx + vy*vy);
}

void Vehicle::updateYaw()
{
  double yaw = atan2(vx, vy);
  this->_yaw = abs(yaw) > 0.1 ? yaw : 0.0;
}

void Vehicle::update(double x, double y, double vx, double vy, double s, double d)
{
  this->_x = x;
  this->_y = y;
  this->_vx = vx;
  this->_vy = vy;
  this->_s = s;
  this->_d = d;
}