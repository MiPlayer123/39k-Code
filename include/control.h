#ifndef __CONTROL_H
#define __CONTROL_H

#include "util.h"
#include "vex.h"
#include "util.h"

using namespace vex;

void setBar(double degs);

void barThread(double degs);

void barT(double t);

void setMogo(double degs);

void mogoThread(double degs);

void mogoPos(int pos, bool daemon);

void startBar(float speed);

void stopBar();

void mogoRotation(float rot);

void spinIntake();

void stopIntake();

void openClaw();

void closeClaw();

void clawSpinT(float t);

#endif