#ifndef __VEHICLE_H__
#define __VEHICLE_H__

#include<vector>
class Vehicle
{
private:
  int _id;
  double _x;
  double _y;
  double _s;
  double _d;
  double _vx;
  double _vy;
  double _yaw;
  double _speed;
  int _lane;

public:
  Vehicle(int id);
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d);
  int id() { return _id; };
  void id(int id) { _id = id; };
  double x() { return _x; };
  void x(double x) { _x = x; };
  double y() { return _y; };
  void y(double y) { _y = y; };
  double s() { return _s; };
  void s(double s) { _s = s; };
  double d() { return _d; };
  void d(double d) { _d = d; };
  double vx() { return _vx; };
  void vx(double vx) { _vx = vx; };
  double vy() { return _vy; };
  void vy(double vy) { _vy= vy; };
  double yaw() { return _yaw; };
  void yaw(double yaw) { _yaw= yaw; };
  double speed() { return _speed; };
  void speed(double speed) { _speed= speed; };
  int lane() { return _lane; };
  void update(double x, double y, double speed, double s, double d);
  void updateYaw();
  void updateLane();
  void updateSpeed();
  double getDistance(double s);
  std::vector<Vehicle*> getClosestCars(std::vector<Vehicle> &vehicles);

};

#endif // __VEHICLE_H__
