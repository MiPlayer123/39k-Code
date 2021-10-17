#include "chasis.h"

#define TURN_KP 0.05
#define TURN_KI 0.002
#define TURN_KD 0.001
#define TURN_MAX_A (BASE_MAX_V / 0.1)
#define TURN_MAX_V (BASE_MAX_V * 0.7)
#define TURN_MIN_V 3

#define   kp 1 // Kp
#define   ki .0 // Ki
#define   kd  0.5 // Kd
#define integral_threshold 10

mutex heading_mtx;

// Filter to track our rotation
Kalman1D heading_filter(1.0, 0.50, 0.0, 0.0);

void initialize() {
// Start a daemon to update the filter in the background
  thread([]() {
    // Set our initial rotation to 0 regardless of the position of the robot
    Inertial.setRotation(0, degrees);
    
    while(true) {
      // Obtain our current rotation according to the sensor
      double measured_rotation = Inertial.rotation();

      // Update the filter
      heading_mtx.lock();
      heading_filter.update(measured_rotation);
      heading_mtx.unlock();

      // Wait 5ms before the next update
      wait(5, msec);
    }
  })
  // Allow the process to run in the background
  .detach();
}

// Get the current rotation of the robot by querying the state of the filter
double get_rotation() {
  heading_mtx.lock();
  double rotation = heading_filter.state;
  heading_mtx.unlock();
  return rotation;
}

void brake_unchecked() {
  BaseLeftRear.stop(brakeType::brake);
  BaseLeftFront.stop(brakeType::brake);
  BaseRightRear.stop(brakeType::brake);
  BaseRightFront.stop(brakeType::brake);
}

// Turn to an absolute rotation
void turn_absolute_inertial(double target) {
  double last_error = 1000;
  double last_output = 0;
  double integral = 0;

  //long ticks = 0;
  long brake_cycles = 0;

  while (true) {
    // Calculate the error
    double error = target - get_rotation();

    // Initialize the last error if it was not already
    if (ae(last_error, 1000)) {
      last_error = error;
    }

    // Calculate the integral and derivative if we're within the bound
    double derivative;
    if (fabs(error) > 10) {
      integral = 0;
      derivative = 0;
    } else {
      integral += error * BASE_DT;
      derivative = (error - last_error) / BASE_DT;
    }
    
    // Calculate the raw output and update the last error
    double raw_output = TURNING_RADIUS * (TURN_KP * error + TURN_KI * integral + TURN_KD * derivative); // in/s
    last_error = error;

    // Constrain acceleration between iterations and calculate the output
    double acceleration = clamp((raw_output - last_output) / BASE_DT, -TURN_MAX_A, TURN_MAX_A); // in/s/s
    double output = last_output + acceleration * BASE_DT;

    // Constrain the maximum velocity
    // Maximum velocity we can have in order to decelerate to min_veloc with max_accel
    double vmax = sqrt(sq(BASE_MIN_V) + 2 * TURN_MAX_A * fabs(error));

    // Constrain this theoretical maximum with the given bounds
    vmax = clamp(vmax, BASE_MIN_V + EPSILON, 100);
    last_output = (output = clamp(output, -vmax, vmax));

    // Constrain the minimum velocity
    output = ithreshold(output, TURN_MIN_V);
    output = clamp(output / MOTOR_PERCENT_TO_IN_PER_SEC, -80, 80);

    // 
    if (fabs(error) < 1) {
      brake_unchecked();
      brake_cycles += 1;
    } else {
      spin(&BaseLeftRear, output);
      spin(&BaseLeftFront, output);
      spin(&BaseRightRear, -output);
      spin(&BaseRightFront, -output);
      brake_cycles = 0;
    }
    if (brake_cycles > 10 && fabs(error) < 1) {
      break;
    }
    wait(BASE_DT, sec);
  }
  BaseLeftFront.stop(vex::brakeType::brake);
  BaseRightFront.stop(vex::brakeType::brake);
  BaseRightRear.stop(vex::brakeType::brake);
  BaseLeftRear.stop(vex::brakeType::brake);
}

void turn_rel_inertial(double target) {
  turn_absolute_inertial(get_rotation() + target);
}

void inertialDrive(double target, double speed){
	double lastError = 0;
	double errorD = 0;
	double _integral = 0;
	double _derivative = 0;
	double leftPos = 0;
	double rightPos = 0;
	double avg = 0;
	double pwrD = 0;

  BaseRightRear.setPosition(0, degrees); 
  BaseLeftRear.setPosition(0, degrees);

	double h0 = get_rotation();
	double pwrT = 0;
	double errorT = 0;
	double _tDerivative = 0;
	double _tIntegral = 0;
	double lastErrorT = 0;

  //reset motor encoders

	while(true){
		leftPos = BaseLeftRear.position(degrees);
		rightPos = BaseRightRear.position(degrees);
		avg = (leftPos + rightPos) / 2;
		errorD = target/TURNS_TO_INCHES*360 - avg;
		_integral += errorD;
		_derivative = errorD - lastError;
		pwrD = (kp * errorD) + (ki * _integral) + (kd * _derivative);

		errorT = h0 - get_rotation();
		_tIntegral += errorT;
		_tDerivative = errorT - lastErrorT;
		pwrT = (.1 * errorT) + (0 * _tIntegral) + (.2 * _tDerivative);
		//drive power is limited to allow turn power to have an effect
		if(pwrD > speed) pwrD = speed;
		else if(pwrD < -speed) pwrD = -speed; 
    BaseLeftFront.spin(vex::directionType::fwd, pwrD + pwrT, vex::velocityUnits::pct);
    BaseLeftRear.spin(vex::directionType::fwd, pwrD + pwrT, vex::velocityUnits::pct);
    BaseRightFront.spin(vex::directionType::fwd, pwrD - pwrT, vex::velocityUnits::pct);
    BaseRightRear.spin(vex::directionType::fwd, pwrD - pwrT, vex::velocityUnits::pct);
		lastError = errorD;
		lastErrorT = errorT;
		if(std::abs(errorD) <= 5){
      BaseLeftFront.stop(vex::brakeType::brake);
      BaseRightFront.stop(vex::brakeType::brake);
      BaseRightRear.stop(vex::brakeType::brake);
      BaseLeftRear.stop(vex::brakeType::brake);
			break;
		}
	}
}

void moveRot (float rot, float speed)
{
  BaseLeftRear.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseLeftFront.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightRear.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightFront.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, true);
}

/*
void inertial_drive(double target, double speed=50.0) {
  BaseRightRear.setPosition(0, turns); 
  BaseLeftRear.setPosition(0, turns);

  //Starting pos
  double angle = get_rotation();

  // Accumulated error
  double integral_c = 0;
  double last_error;
  double derivative;
  double integral;

  while (true) {
    // Calculate the error
    double error_c = angle - get_rotation();
    double error1 = target - BaseRightRear.position(turns) * TURNS_TO_INCHES;
    double error2 = target - BaseLeftRear.position(turns) * TURNS_TO_INCHES;
    double error = (error1 + error2) / 2;

    // Get the turn output
    double raw_output_correct = (TURN_KP * error_c + 0.1 * integral_c); // in/s
    integral_c += error_c * BASE_DT;

    if (fabs(error) > integral_threshold) {
      integral = 0;
      derivative = 0;
    } else {
      integral += error * BASE_DT;
      derivative = (error - last_error) / BASE_DT;
    }
    
    double raw_output = kp * error + ki * integral + kd * derivative; // in/s
    last_error = error;

    // This is the extent to which we want to turn; a low value means no turns
    double factor = 0.1 + fabs(error_c) / 45;
    raw_output_correct *= factor;
    //double correct_output = 2 * clamp(raw_output_correct, -speed, speed);

    if(raw_output > speed) raw_output = speed;
		else if(raw_output < -speed) raw_output = -speed; 
    BaseLeftFront.spin(vex::directionType::fwd, raw_output + raw_output_correct, vex::velocityUnits::pct);
    BaseLeftRear.spin(vex::directionType::fwd, raw_output + raw_output_correct, vex::velocityUnits::pct);
    BaseRightFront.spin(vex::directionType::fwd, raw_output - raw_output_correct, vex::velocityUnits::pct);
    BaseRightRear.spin(vex::directionType::fwd, raw_output - raw_output_correct, vex::velocityUnits::pct);

		if(std::abs(error) <= .5){
		  BaseLeftFront.stop(vex::brakeType::brake);
      BaseRightFront.stop(vex::brakeType::brake);
      BaseRightRear.stop(vex::brakeType::brake);
      BaseLeftRear.stop(vex::brakeType::brake);
			break;
		}

    // Apply the adjustment to the linear PID controllers
    //base_left.set_adjustment(output);
    //base_right.set_adjustment(-output);

    // Wait until the next update
    wait(BASE_DT, sec);
  }
}
*/
/* Teporarily gone

// PID controllers for the left and right side of the base
DualPidController base_left(
  &BaseLeftRear,
  &BaseLeftFront,
  2.25, // Kp
  1.1, // Ki
  0.7, // Kd
  BASE_DT,
  BASE_INTEGRAL_THRESHOLD
);
DualPidController base_right(
  &BaseRightRear,
  &BaseRightFront,
  2.25, // Kp
  1.1, // Ki
  0.7, // Kd
  BASE_DT,
  BASE_INTEGRAL_THRESHOLD
);

// Control variable to set whether or not the base PIDs should update
volatile bool tick_pids = false;

// Filter to track our rotation
Kalman1D heading_filter(1.0, 0.50, 0.0, 0.0);

// Mutual exclusions structs for thread safety
mutex chasis_mtx;
mutex heading_mtx;

// Hit the brakes without checking whether or not the PIDs are running
void brake_unchecked() {
  BaseLeftRear.stop(brakeType::brake);
  BaseLeftFront.stop(brakeType::brake);
  BaseRightRear.stop(brakeType::brake);
  BaseRightFront.stop(brakeType::brake);
}

// Set the PID constraints
void set_constraints(double min_veloc, double max_veloc, double max_accel) {
  chasis_mtx.lock();
  // Start with no turn correction
  base_left.set_adjustment(0);
  base_right.set_adjustment(0);
  // Apply new constraints
  base_left.set_constraints(min_veloc, max_veloc, max_accel);
  base_right.set_constraints(min_veloc, max_veloc, max_accel);
  chasis_mtx.unlock();
}

void initialize() {
  // Set the PID constraints
  set_constraints(BASE_MIN_V, 100, BASE_MAX_A);
  
  // Thread to update the PIDs
  thread([]() {
    double last_tick_pids = false;
    long ticks = 0;
    while(true) {
      chasis_mtx.lock();

      // If we should tick the PIDs, then update them
      if (tick_pids) {
        base_left.update();
        base_right.update();
      }
      // If we just turned off the PIDs, then hit the brakes
      else if (last_tick_pids) {
        brake_unchecked();
      }

      // Keep track of whether we updated the last tick
      last_tick_pids = tick_pids;

      chasis_mtx.unlock();
      
      ticks += 1;
      wait(BASE_DT, sec);
    }
  })
  // Allow the process to run in the background
  .detach();

  // Start a daemon to update the filter in the background
  thread([]() {
    // Set our initial rotation to 0 regardless of the position of the robot
    Inertial.setRotation(0, degrees);
    
    while(true) {
      // Obtain our current rotation according to the sensor
      double measured_rotation = Inertial.rotation();

      // Update the filter
      heading_mtx.lock();
      heading_filter.update(measured_rotation);
      heading_mtx.unlock();

      // Wait 5ms before the next update
      wait(5, msec);
    }
  })
  // Allow the process to run in the background
  .detach();
}

// Get the current rotation of the robot by querying the state of the filter
double get_rotation() {
  heading_mtx.lock();
  double rotation = heading_filter.state;
  heading_mtx.unlock();
  return rotation;
}

// Stop running the base PIDs
void unlock() {
  chasis_mtx.lock();
  tick_pids = false;
  chasis_mtx.unlock();
}

// Apply the brakes after disabling the PIDs
void apply_brake() {
  unlock();
  brake_unchecked();
}

// Move a relative distance
void move_rel(double left, double right, double max_time) {
  // Update the PIDs' target values
  chasis_mtx.lock();
  base_left.set_rel_target(left);
  base_right.set_rel_target(right);
  tick_pids = true;
  chasis_mtx.unlock();

  double start_time = cts();
  long ticks = 0;

  // Run until we reach our target or hit our time out
  while(max_time == 0 || cts() - start_time < max_time) {
    chasis_mtx.lock();

    //if (ticks % 20 == 0) {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("%s, %s", base_left.is_done() ? "true" : "false", base_right.is_done() ? "true" : "false");
    //}

    // If we reached our target, then hit the brakes
    if (base_left.is_done() && base_right.is_done()) {
      brake_unchecked();
      tick_pids = false;
      chasis_mtx.unlock();
      break;
    }

    chasis_mtx.unlock();

    wait(20, msec);
    ticks += 1;
  }
}

// Hold the robot in its current position
void lock() {
  move_rel(0, 0);
}
*/