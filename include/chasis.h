#ifndef __BOT_H
#define __BOT_H

#include "kalman.h"
#include "control.h"
#include <string>

#define BASE_DT 0.005
#define BASE_INTEGRAL_THRESHOLD 20

#define BASE_MIN_V 3 // in/s
#define BASE_MAX_V 45 // in/s
#define BASE_MAX_A (BASE_MAX_V / 0.1) // in/s/s

extern double xPos;
extern double yPos;

// Initialize the chasis. Must be called once and only once at the start of the program.
void initialize();

double get_rotation();

void brake_unchecked();

// Turns the robot to an absolute rotation.
void turn_absolute_inertial(double target, bool heavy=false, std::string swing="None");

// Turns the robot a relative number of degrees.
void turn_rel_inertial(double target);

//PID drive with inertial correction
void inertial_drive(double target, double speed, bool dist=false);

//Move given rotations
void moveRot (float rot, float speed);

//Turn Rotations
void turnRot (float rot, float speed, std::string swing="None");

//Drive to point using odom position
void driveTo(double xTarget, double yTarget, double targetAngle, double speed);

//Automaticalyy balance robot on platform
void autobalance();

//Base voltage   
void allBaseVoltage(bool Dir, double v);

//Move backwards voltage to distance
void voltageDist(double dist);

//Drives to a goal and grabs it
void driveToGoal(float volt=7);

//Trial inertial drive
void inertialDrive(double target, double speed); 

#endif