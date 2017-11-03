#ifndef __VEHICLE_H__
#define __VEHICLE_H__

class Vehicle
{
private:
  int _id;
  double _x;
  double _y;
  double _vx;
  double _vy;
  double _s;
  double _d;
  double _yaw;
  double _velocity;
  int _lane;

public:
  Vehicle(double id, double x, double y, double vx, double vy, double s, double d);
  double x() { return _x; };
  double y() { return _y; };
  double vx() { return _vx; };
  double vy() { return _vy; };
  double s() { return _s; };
  double d() { return _d; };
  int lane() { return _lane; };
  double velocity() { return _velocity; };
  double yaw() { return _yaw; };
  void update(double x, double y, double vx, double vy, double s, double d);
  void updateYaw();
  void updateLane();
  void updateVelocity();

};

#endif // __VEHICLE_H__
