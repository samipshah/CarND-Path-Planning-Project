#ifndef COMMON_HPP
#define COMMON_HPP
#include <cmath>

constexpr double pi() { return M_PI; }
double deg2rad(double x); 
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);

#endif /* COMMON_HPP */