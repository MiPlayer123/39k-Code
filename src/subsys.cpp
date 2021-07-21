#include "subsys.h"
#include "util.h"

void drive(double left, double right, vex::brakeType brake_type) {
  spin(&BaseLeftRear, left, brake_type);
  spin(&BaseLeftFront, left, brake_type);
  spin(&BaseRightRear, right, brake_type);
  spin(&BaseRightFront, right, brake_type);
}

task wiggle() {
  return task([]() {
    while(true) {
      drive(50, 50);
      wait(0.7, sec);
      drive(0, 0, coast);
      wait(0.1, sec);
      drive(-50, -50);
      wait(0.2, sec);
      drive(0, 0, coast);
      wait(0.1, sec);
    }
    return 0;
  });
}