#include "consts.h"
#include <cmath>

const double EPSILON = 1e-5;
const double TRACKING_POINT_TO_ODO_LEFT = 7.48;
const double TRACKING_POINT_TO_ODO_RIGHT = 7.48;
const double TRACKING_POINT_TO_ODO_DRIFT = 5.12;
const double TURNS_TO_INCHES = 4.397;
const double MOVEMENT_THRESHOLD = 0.05;
const double THETA_THRESHOLD = 0.01;
const double M_2PI = 2.0 * M_PI;
const double TURNING_RADIUS = 7.90; // old: 7.768
const double LIGHT_SENSOR_GAP = 5.551;
const double RPM_TO_INCHES_PER_SEC = TURNS_TO_INCHES / 60.0;
const double MOTOR_PERCENT_TO_IN_PER_SEC = 0.436;