#ifndef __HELPER_H__
#define __HELPER_H__

#include <math.h>

inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double vxvy2speed(double vx, double vy) { return sqrt(vx*vx + vy*vy); }

inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


#endif // __HELPER_H__