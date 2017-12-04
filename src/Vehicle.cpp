#include "Vehicle.h"
#include "constants.h"
#include "helper.h"
#include <math.h>
  
using namespace std;

Vehicle::Vehicle(int id) : _id(id) {}; 

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d) : _id(id), _x(x), _y(y), _vx(vx), _vy(vy), _s(s), _d(d) 
{
  this->updateSpeed();
  this->updateLane();
  this->updateYaw();
}

void Vehicle::updateLane()
{
  this->_lane = (int) this->_d / LANE_WIDTH;
}

void Vehicle::updateSpeed()
{
  this->_speed = vxvy2speed(_vx, _vy);
}

void Vehicle::updateYaw()
{
  double yaw = atan2(_vx, _vy);
  this->_yaw = fabs(yaw) > 0.1 ? yaw : 0.0;
}

void Vehicle::update(double x, double y, double speed, double s, double d)
{
  this->_x = x;
  this->_y = y;
  this->_speed = speed;
  this->_s = s;
  this->_d = d;
}
/**
 *
 * @param s - S to get distance to
 * @return Distance to S
 */
double Vehicle::getDistance(double s) 
{
  if (s < (this->_s - CAR_SIZE)) {
      s += TRACK_SIZE;
  }
  return s - this->_s;
}