#ifndef __CONTROL_H
#define __CONTROL_H

#include "util.h"
#include "vex.h"
#include "util.h"

using namespace vex;

void setBar(double rot);

void spinBar(double speed);

void stopBar();

void openClaw();

void closeClaw();

void clawSpinT(float t);

void mogoSpin(float rot);

void moveRearFork(double rot);

void moveFronFork(double rot);

#endif