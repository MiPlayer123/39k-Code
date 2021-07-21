#ifndef __UTIL_H
#define __UTIL_H

#include <cmath>
#include <ctime>
#include "vex.h"
#include "consts.h"

enum Ball {
  Red,
  Blue,
  None
};

// Spin the given motor with the given power (in units of percent). If the power
// is very close to 0 then the brake is applied.
void spin(motor *mot, double veloc, vex::brakeType brake_type=brake);

// Interpret the data from an optical sensor as either a red ball, blue ball, or
// no ball.
Ball read_optical(optical *sensor);

// Our team color.
Ball team_color();

// Red -> Blue, Blue -> Red, and None -> None.
Ball invert_color(Ball color);

// Whether or not we're running skills.
bool is_skills();

// Returns true if a and b are within epsilon of each other as defined in consts.h.
bool ae(double a, double b);

// Clamps x between the given minimum and maximum.
double clamp(double x, double mn, double mx);

double iclamp(double x, double lim);

// Performs an inverse threshold, meaning that if |x| < threshold then threshold is
// returned.
double ithreshold(double x, double threshold);

// Clamps an angle (in radians) between -pi and pi.
double clamp_angle(double theta);

// Returns the current system time in seconds.
long double cts();

// Returns the sign of x: 1 if positive, -1 if negative, or 0 if x == 0.
int sign(double x);

// Returns the square of x.
double sq(double x);

// If |x| < threshold, then 0 is returned, else x is returned.
double threshold(double x, double threshold);

#endif