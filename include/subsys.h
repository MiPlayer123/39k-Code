#ifndef __SUBSYS_H
#define __SUBSYS_H

#include "vex.h"
#include "util.h"

// Power the base with the given left and right percentages
void drive(double left, double right, vex::brakeType brake_type=brake);

// Have the base oscillate between moving forward and backward
task wiggle();

#endif