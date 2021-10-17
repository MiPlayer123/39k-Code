#include "control.h"
#include "chasis.h"

using namespace vex;

//Voids for auton 

void setBar(double t){
  //P loop 4bar height
  /*
  double original = Bar.position(degrees);
  double error;
  
  while(true){
    error = rot - (Bar.position(degrees)/ - original);
    Bar.spin(fwd,error*1,pct);
    if(error<.1){
      Bar.stop(brake);
      break;
    }
  */
  Bar.spin(fwd,100,pct);
  vex::task::sleep(t*1000);
  Bar.stop(hold);
}

void closeClaw(){
  //open claw
  double degOpen = 270;
  double error;
  while(true){
    error = degOpen - Claw.position(degrees);
    Claw.spin(fwd,error*1,pct);
    if(error<.1){
      Claw.stop(brake);
      break;
    }
  }
}

void openClaw(){
  //Close claw
  double error;
  while(true){
    error = 0 - Claw.position(degrees);
    Claw.spin(fwd,error*1,pct);
    if(error<.1){
      Claw.stop(brake);
      break;
    }
  }
}

void clawSpin(float rot){
  Claw.rotateFor(rot, rotationUnits::rev, 80, velocityUnits::pct, true);
}

void moveFrontFork(double rot){
  //raise or lower fork
  double original = FrontMogo.position(degrees)/360;
  double error;
  while(true){
    error = rot - (FrontMogo.position(degrees)/360 - original);
    FrontMogo.spin(fwd,error*1,pct);
    if(error<.1){
      FrontMogo.stop(hold);
      break;
    }
  }
}

void moveRearFork(double rot){
  //raise or lower fork
  double original = RearMogo.position(degrees)/360;
  double error;
  while(true){
    error = rot - (RearMogo.position(degrees)/360 - original);
    RearMogo.spin(fwd,error*1,pct);
    if(error<.1){
      RearMogo.stop(hold);
      break;
    }
  }
}

/* Temporarily gone

#define LAST_ERROR_NICHE 1000

DualPidController::DualPidController(
  motor *m1,
  motor *m2,
  double kp,
  double ki,
  double kd,
  double dt,
  double integral_threshold
) : m1(m1), m2(m2), kp(kp), ki(ki), kd(kd), dt(dt), integral_threshold(integral_threshold)
{
  // Default to (basically) no constraints
  min_veloc = 0;
  max_veloc = 100;
  max_accel = 10000;

  target = 0;
  initial_error = 10000;
  last_error = LAST_ERROR_NICHE;
  integral = 0;
  last_output = 0;
  adjustment = 0;
  brake_ticks = 0;
}

void DualPidController::set_constraints(double minv, double maxv, double maxa) {
  min_veloc = minv;
  max_veloc = maxv;
  max_accel = maxa;
}

void DualPidController::set_rel_target(double new_target) {
  target = new_target;
  initial_error = 10000;
  last_error = LAST_ERROR_NICHE;
  integral = 0;
  last_output = 0;
  brake_ticks = 0;
  m1->setPosition(0, turns);
  m2->setPosition(0, turns);
}

double DualPidController::error() {
  return last_error;
}

bool DualPidController::is_done() {
  return brake_ticks > 10;
}

void DualPidController::set_adjustment(double adj_value) {
  adjustment = adj_value;
}

void DualPidController::update(bool debug) {
  // Pid loop
  double error1 = target - m1->position(turns) * TURNS_TO_INCHES;
  double error2 = target - m2->position(turns) * TURNS_TO_INCHES;
  double error = (error1 + error2) / 2;
  double veloc = (m1->velocity(pct) + m2->velocity(pct)) * MOTOR_PERCENT_TO_IN_PER_SEC / 2.0;

  if (ae(last_error, LAST_ERROR_NICHE)) {
    last_error = error;
    initial_error = error;
  }

  // Only apply the integral and derivative components if we're within a
  // certain range of our set point.
  double derivative;
  if (fabs(error) > integral_threshold) {
    integral = 0;
    derivative = 0;
  } else {
    integral += error * dt;
    derivative = (error - last_error) / dt;
  }
  
  double raw_output = kp * error + ki * integral + kd * derivative; // in/s
  last_error = error;

  // Constrain acceleration between iterations and calculate the output
  double acceleration = clamp((raw_output - last_output) / dt, -max_accel, max_accel); // in/s/s
  double output = last_output + acceleration * dt;

  // Constrain the maximum velocity
  // Maximum velocity we can have in order to decelerate to min_veloc with max_accel
  double raw_vmax = sqrt(sq(min_veloc) + 2 * max_accel * fabs(error));

  // Constrain this theoretical maximum with the given bounds
  double vmax = clamp(raw_vmax, min_veloc + EPSILON, max_veloc);
  last_output = (output = clamp(output, -vmax, vmax));

  // Constrain the velocity between the hard maximum and minimum values
  output = clamp(output / MOTOR_PERCENT_TO_IN_PER_SEC, -100, 100);

  if ((sign(error) != sign(initial_error) && fabs(error) < 1) || fabs(error) < 0.35) {
    brake_ticks += 1;
    output = 0;
  }

  spin(m1, output);
  spin(m2, output);
}
*/