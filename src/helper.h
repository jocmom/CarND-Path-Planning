#ifndef __HELPER_H__
#define __HELPER_H__

#include <math.h>

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

#endif // __HELPER_H__