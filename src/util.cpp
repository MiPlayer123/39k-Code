#include "util.h"

void spin(motor *mot, double veloc, vex::brakeType brake_type) {
  if (ae(veloc, 0)) {
    mot->stop(brake_type);
  } else if (veloc < 0) {
    mot->spin(directionType::rev, fabs(veloc), percent);
  } else {
    mot->spin(directionType::fwd, veloc, percent);
  }
}

bool is_skills() {
  return Skills.pressing();
}

bool ae(double a, double b) {
  return fabs(a - b) < EPSILON;
}

double clamp(double x, double mn, double mx) {
  if (x > mx) {
    return mx;
  } else if (x < mn) {
    return mn;
  }

  return x;
}

double iclamp(double x, double lim) {
  if (x > -lim && x < 0) {
    return -lim;
  } else if (x < lim && x > 0) {
    return lim;
  }
  return x;
}

double ithreshold(double x, double threshold) {
  if (x < 0 && x > -threshold) {
    return -threshold;
  } else if (x > 0 && x < threshold) {
    return threshold;
  } else {
    return x;
  }
}

double clamp_angle(double theta) {
  theta = fmodf(theta, M_2PI);

  if (theta < -M_PI) {
    return theta + M_2PI;
  } else if (theta > M_PI) {
    return theta - M_2PI;
  }

  return theta;
}

long double cts() {
  long double current_time = (long double)clock();
  return current_time / 10.0;
}

int sign(double x) {
  return x > 0 ? 1 : (x < 0 ? -1 : 0);
}

double sq(double x) {
  return x * x;
}

double threshold(double x, double threshold) {
  if (fabs(x) < threshold) {
    return 0;
  } else {
    return x;
  }
}