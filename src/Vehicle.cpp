#include "Vehicle.h"
#include "constants.h"
#include <math.h>

using namespace std;

Vehicle::Vehicle(double id, double x, double y, double vx, double vy, double s, double d) : 
id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d)
{
}

int Vehicle::getLane()
{
  return (int) this->d / LANE_WIDTH;
}

double Vehicle::getVelocity()
{
  return sqrt(vx*vx + vy*vy);
}