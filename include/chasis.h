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

// Set the constraints for the linear movement PID controllers.
void set_constraints(double min_veloc, double max_veloc, double max_accel);

// Move a relative distance (in inchest).
void move_rel(double left, double right, double max_time=0);

// Hold the base in its current position. This is different from braking since the PID controllers
// will attempt to reset the robots position if it is moved.
void lock();

// Disable the PID controllers. They will be reactivated when move_rel is called again.
void unlock();

// Disable the PID controllers and apply the motor brakes.
void apply_brake();

// Returns the absolute rotation of the robot in degrees.
double get_rotation();

// Turns the robot to an absolute rotation.
void turn_absolute_inertial(double target);

// Turns the robot a relative number of degrees.
void turn_rel_inertial(double target);

void correct_drive(double angle);

#endif