#ifndef __VEHICLE_H__
#define __VEHICLE_H__

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
  double _velocity;
  int _lane;

public:
  Vehicle(int id);
  Vehicle(int id, double x, double y, double vel, double s, double d);
  int id() { return _id; };
  double x() { return _x; };
  double y() { return _y; };
  double s() { return _s; };
  double d() { return _d; };
  double vx() { return _vx; };
  double vy() { return _vy; };
  double velocity() { return _velocity; };
  int lane() { return _lane; };
  double yaw() { return _yaw; };
  void update(double x, double y, double vel, double s, double d);
  void updateYaw();
  void updateLane();
  void updateVelocity();

};

#endif // __VEHICLE_H__
