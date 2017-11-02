#ifndef __VEHICLE_H__
#define __VEHICLE_H__

class Vehicle
{
private:
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;

public:
  Vehicle(double id, double x, double y, double vx, double vy, double s, double d);
  int getLane();
  double getVelocity();

};

#endif // __VEHICLE_H__
