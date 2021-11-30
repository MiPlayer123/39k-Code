#include "chasis.h"
#include "odometry.h"

//For PID turns
#define TURN_KP 0.05
#define TURN_KI 0.002
#define TURN_KD 0.001
#define TURN_MAX_A (BASE_MAX_V / 0.1)
#define TURN_MAX_V (BASE_MAX_V * 0.7)
#define TURN_MIN_V 3

//For main inertial_drive
#define   kp 10 //21.1 // Ki
#define   ki .8 //1.1 // Ki
#define   kd  0.7 // Kd
#define integral_threshold 10
#define kp_c .2

//For other inertialDrive()
#define   m_kp 2 // Kp
#define   m_ki 0 // Ki
#define   m_kd  0.0 // Kd

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

void moveRot (float rot, float speed)
{
  BaseLeftRear.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseLeftFront.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightRear.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, false);
  BaseRightFront.rotateFor(rot, rotationUnits::rev, speed, velocityUnits::pct, true);
}

void inertial_drive(double target, double speed) {
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
    raw_output_correct *= factor; //old error
    raw_output_correct = error_c*kp_c; //new ange error
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
}
}

//Drive to position on field based on odom

//target coords to drive to
double xTargetLocation = xPosGlobal;
double yTargetLocation = yPosGlobal;
double targetFacingAngle = 0;

//distances between robot's current position and the target position
double xDistToTarget = 0;
double yDistToTarget = 0;

//angle of hypotenuse of X and Y distances
double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

double driveError = 0;

void driveTo(double xTarget, double yTarget, double targetAngle, double speed) {

  xDistToTarget = xTargetLocation - xPosGlobal;
  yDistToTarget = yTargetLocation - yPosGlobal;

  //Angle of hypotenuse
  hypotenuseAngle = atan2(yDistToTarget, xDistToTarget);

  if(hypotenuseAngle < 0) {
    hypotenuseAngle += 2 * M_PI;
  }

  //The angle the robot needs to travel relative to its forward direction in order to go toward the target
  robotRelativeAngle = hypotenuseAngle - currentAbsoluteOrientation + M_PI_2;

  if(robotRelativeAngle > 2 * M_PI) {
    robotRelativeAngle -= 2 * M_PI;
  }
  else if(robotRelativeAngle < 0) {
    robotRelativeAngle += 2 * M_PI;
  }

  double turnAngle = hypotenuseAngle*180/M_PI; //Convert to degrees
  turn_absolute_inertial(turnAngle); //Turn to face the point

  //Constrianed Inertial PID for traveling the distance
      
  double angle = get_rotation();

  // Accumulated error
  double integral_c = 0;
  double last_error;
  double derivative;
  double integral;

  while (true) {

    // Calculate the error
    double error_c = angle - get_rotation();
    
    driveError = sqrt(pow((xPosGlobal - xTargetLocation), 2) + pow((yPosGlobal - yTargetLocation), 2));

    // Get the turn output
    double raw_output_correct = (TURN_KP * error_c + 0.1 * integral_c); // in/s
    integral_c += error_c * BASE_DT;

    if (fabs(driveError) > integral_threshold) {
      integral = 0;
      derivative = 0;
    } else {
      integral += driveError * BASE_DT;
      derivative = (driveError - last_error) / BASE_DT;
    }
    
    double raw_output = kp * driveError + ki * integral + kd * derivative; // in/s
    last_error = driveError;

    // This is the extent to which we want to turn; a low value means no turns
    double factor = 0.1 + fabs(error_c) / 45;
    raw_output_correct *= factor; //old error
    raw_output_correct = error_c*kp_c; //new ange error
    //double correct_output = 2 * clamp(raw_output_correct, -speed, speed);

    if(raw_output > speed) raw_output = speed;
		else if(raw_output < -speed) raw_output = -speed; 
    BaseLeftFront.spin(vex::directionType::fwd, raw_output + raw_output_correct, vex::velocityUnits::pct);
    BaseLeftRear.spin(vex::directionType::fwd, raw_output + raw_output_correct, vex::velocityUnits::pct);
    BaseRightFront.spin(vex::directionType::fwd, raw_output - raw_output_correct, vex::velocityUnits::pct);
    BaseRightRear.spin(vex::directionType::fwd, raw_output - raw_output_correct, vex::velocityUnits::pct);

		if(std::abs(driveError) <= .5){
		  BaseLeftFront.stop(vex::brakeType::brake);
      BaseRightFront.stop(vex::brakeType::brake);
      BaseRightRear.stop(vex::brakeType::brake);
      BaseLeftRear.stop(vex::brakeType::brake);
			break;
		}
  }

  if(targetAngle!=-1){ //If we want to face a certain angle
    turn_absolute_inertial(targetAngle);
  }
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

    Controller1.Screen.setCursor(1, 1); 
    Controller1.Screen.print(target/TURNS_TO_INCHES*360); 

		_integral += errorD;
		_derivative = errorD - lastError;
		pwrD = (m_kp * errorD) + (m_ki * _integral) + (m_kd * _derivative);

		errorT = h0 - get_rotation();
		_tIntegral += errorT;
		_tDerivative = errorT - lastErrorT;
		//pwrT = (.1 * errorT) + (0 * _tIntegral) + (.2 * _tDerivative);
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
