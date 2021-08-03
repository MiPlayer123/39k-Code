#ifndef __BOT_H
#define __BOT_H

#include "kalman.h"
#include "control.h"

#define BASE_DT 0.005
#define BASE_INTEGRAL_THRESHOLD 20

#define BASE_MIN_V 3 // in/s
#define BASE_MAX_V 45 // in/s
#define BASE_MAX_A (BASE_MAX_V / 0.1) // in/s/s

// Initialize the chasis. Must be called once and only once at the start of the program.
void initialize();

double get_rotation();

void brake_unchecked();

// Turns the robot to an absolute rotation.
void turn_absolute_inertial(double target);

// Turns the robot a relative number of degrees.
void turn_rel_inertial(double target);

void inertial_drive(double target, double speed);

#endif