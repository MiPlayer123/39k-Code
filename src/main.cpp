/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Ian                                              */
/*    Created:      Thu Sep 10 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// BaseLeftRear         motor         1               
// BaseLeftFront        motor         2               
// BaseRightRear        motor         21              
// BaseRightFront       motor         9               
// UpperRoller          motor         7               
// LowerRoller          motor         4               
// LeftIntake           motor         5               
// RightIntake          motor         10              
// Prog0                bumper        A               
// ReverseTurns         bumper        G               
// Prog1                bumper        B               
// Inertial             inertial      15              
// Skills               bumper        H               
// IsRed                bumper        F               
// BallSensor           optical       17              
// CycleTrigger         bumper        E               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "vex_controller.h"
#include "consts.h"
#include "kalman.h"
#include "control.h"
#include "chasis.h"
#include "subsys.h"

using namespace vex;

// Subroutine for a scoring a ball in a goal
void dump_ball(double back_up, double intake_speed=0) {
  // Start scoring the ball
  rollers(OutTop);

  // Intaking helps move the ball along
  intake(In);
  wait(0.25, sec);

  // Make sure we don't intake the second ball
  intake(Off);
  wait(0.75, sec);

  // Stop the rollers
  rollers(Off);

  // Get any other balls out of the rollers
  intake(Out, intake_speed);
  rollers(OutFront);

  // Back up from the goal
  move_rel(-back_up, -back_up);

  // Stop outtaking
  intake(Off);
  rollers(Off);
}

// Score one goal
void match_auton_1(bool reverse_turns) {
  double turn_factor = reverse_turns ? -1 : 1;
  
  // Release the flap so the driver doesn't have to
  release_flap();

  // Move up to the goal
  move_rel(18, 18);
  turn_absolute_inertial(turn_factor * -135);
  move_rel(18, 18, 1.5);
  
  unlock();

  // Score preload
  cycle_n(1);
  task wiggle_task = wiggle();

  // Intake ball in front of goal
  intake_mid();
  // Intake lower red ball
  intake_low();

  wiggle_task.suspend();
  drive(0, 0);

  // Lower the balls in the roller-intake system
  lower_rollers();

  // Score the ball in front of the goal
  cycle_n(1);

  wiggle_task.resume();

  // Take lower red ball from intakes to rollers
  intake_mid();
  // Take blue ball
  intake_low();

  wiggle_task.stop();
  drive(0, 0);

  // Lower the balls in the roller-intake system
  lower_rollers();

  // Score the last red ball
  cycle_n(1);

  // Back up from the goal
  move_rel(-8, -8);
}

void match_auton_2(bool reverse_turns) {
  // Multiplying by negative one reverses the turns
  double turn_factor = reverse_turns ? -1 : 1;

  // Score the middle goal
  release_flap();
  wait(1.5, sec);

  // Move to the corner goal and intake the ball in front of it
  intake(In);
  move_rel(36, 36);
  turn_absolute_inertial(turn_factor * 135);
  move_rel(24, 24, 2.5);
  
  unlock();

  task wiggle_task = wiggle();

  // Intake ball in front of goal
  intake_mid();
  // Intake lower red ball
  intake_low();

  wiggle_task.suspend();
  drive(0, 0);

  // Lower the balls in the roller-intake system
  lower_rollers();
  // Score the ball in front of the goal
  cycle_n(2);

  wiggle_task.resume();

  // Intake the lower blue ball
  intake_mid();

  wiggle_task.stop();
  intake(Out);

  // Back up from the goal
  move_rel(-12, -12);
}

void match_auton_2_corner_middle(bool reverse_turns) {
  double turn_factor = reverse_turns ? -1 : 1;

  move_rel(18, 18);
  turn_absolute_inertial(turn_factor * -135);
  move_rel(18, 18, 1.5);
  
  unlock();

  // Score preload
  cycle_n(1);

  // Helps get the robot far enough into the goal
  task wiggle_task = wiggle();

  // Intake ball in front of goal
  intake_mid();
  // Intake lower red ball
  intake_low();

  wiggle_task.suspend();
  drive(0, 0);

  // Lower the balls in the roller-intake system
  lower_rollers();
  // Score the ball in front of the goal
  cycle_n(1);

  wiggle_task.resume();

  // Take lower red ball
  intake_mid();
  // Take blue ball
  intake_low();

  wiggle_task.stop();
  drive(0, 0);

  task([]() {
    // Outtake the blue ball if it's in the intakes
    intake(Out);
    wait(1, sec);

    // Make sure it's not stuck in the rollers
    rollers(OutFront);
    waitUntil(read_optical(&BallSensor) == Red);

    // Finish discarding the blue ball
    intake(Off);
    rollers(Off);
    return 0;
  });

  // Back up from the goal
  move_rel(-12, -12);

  // Re-angle ourselves and drive up to the center goal
  turn_absolute_inertial(turn_factor * -120);
  move_rel(-44, -44);

  // Turn around towards the goal and score the ball
  turn_absolute_inertial(turn_factor * 20);
  move_rel(20, 20);

  // Score the ball
  intake(In);
  rollers(OutTop);
  wait(2, sec);
  intake(Off);
  rollers(Off);

  // Back up from the center goal
  move_rel(-10, -10);
}

void match_auton_home_row() {
  // Release the flap and score the middle goal
  release_flap();
  wait(1, sec);

  // Move to the corner goal and intake the ball
  move_rel(36, 36);
  turn_absolute_inertial(135);
  intake(In);
  move_rel(25, 25, 1.5);

  // Score the corner goal
  dump_ball(30, 100);

  // Drive to the other side of the field with drift correction
  turn_absolute_inertial(270);
  task correction = task([]() {
    // Ensure that we drive at an angle of 270 degrees
    correct_drive(270);
    return 0;
  });
  move_rel(87, 87);
  correction.stop();

  // Drive to the other corner goal and intake the ball
  turn_absolute_inertial(225);
  intake(In);
  move_rel(30, 30, 1.5);

  // Score the other corner goal
  dump_ball(10, 100);
}

void skills2() {
  /* Ball 1 */

  release_flap();
  wait(1, sec);

  /* Ball 2 */

  // Grab the ball
  intake(In);
  move_rel(25, 25);

  // Turn parallel to the wall goal and move to goal
  turn_absolute_inertial(-90);
  move_rel(34, 34);

  // Turn towards the goal and throw the ball in
  turn_absolute_inertial(-180);
  intake(Off);
  move_rel(10, 10, 0.8);
  dump_ball(8, 100);

  /* Ball 3 */

  // Turn towards the corner goal and move to it
  turn_absolute_inertial(-92);
  intake(In);
  move_rel(48.5, 48.5);

  // Turn towards the goal and move into it
  turn_rel_inertial(-45);
  intake(Off);
  move_rel(18, 18, 1.2);

  dump_ball(31.5, 100);

  /* Ball 4 */

  // Turn towards the ball on the wall, grab it and return back to original position
  turn_absolute_inertial(-88);
  task intake_task = task([]() {
    intake_mid();
    return 0;
  });

  // Drive up to the ball along the wall and drive back
  move_rel(23, 23);
  move_rel(-23, -23);
  intake_task.stop();

  // Turn towards the ball next to the goal and move to it
  turn_absolute_inertial(-19);
  intake_task = task([]() {
    intake_low();
    return 0;
  });
  move_rel(41, 41);
  intake_task.stop();

  // Get the balls into the ideal position in the rollers
  task([]() {
    lower_rollers(0.3);
    intake(Out);
    wait(0.15, sec);
    intake(Off);
    return 0;
  });

  // Move into the goal on the wall
  turn_absolute_inertial(-90);
  intake(Off);
  move_rel(11, 11, 1);

  // Score the ball
  cycle_n(1);
  move_rel(-8, -8);

  /* Ball 5 */

  // If we have another red ball in our rollers, then go to the next goal
  if (intake_low(1.5)) {
    // Turn to be aligned with the goal
    turn_absolute_inertial(-6.34);

    // This helps unjam the intakes if they're jammed
    task([]() {
      intake(Out);
      wait(0.15, sec);
      intake(In);
      return 0;
    });

    // Move to the next goal
    move_rel(52, 52);

    // Turn towards second corner and drive up to it
    turn_absolute_inertial(-45);
    intake(Off);
    move_rel(10, 10, 1);

    // Score the ball
    dump_ball(8, 100);
  } else {
    // Turn around and get the ball behind us
    turn_absolute_inertial(90);
    intake(In);
    move_rel(26, 26);

    // Turn towards the corner goal and drive up to it
    turn_absolute_inertial(-34);
    intake(Off);
    move_rel(70, 70, 3);

    // Score the ball
    dump_ball(8, 100);
  }

  /* Ball 6 */

  // Turn towards the next ball (we need a larger angle to compensate
  // for the odd angle with which we approached the corner goal)
  turn_absolute_inertial(105);
  intake(In);

  // Drive up to the line perpendicular through the goal
  move_rel(59, 59);

  // Turn towards the goal and drive up to it
  turn_absolute_inertial(0);
  move_rel(20, 20, 1.25);

  // Score the goal and back up from it
  dump_ball(8, 100);

  /* Ball 7 */

  // Turn towards the goal and drive up to it
  turn_absolute_inertial(90);
  intake(In);
  move_rel(48, 48);

  // Turn into the goal and drive up to it
  turn_absolute_inertial(45);
  intake(Off);
  move_rel(18, 18, 1);

  // Score the goal and back up from it
  dump_ball(18, 100);

  /* Ball 8 */

  // Turn towards the ball in front of the goal and intake it
  turn_absolute_inertial(183);
  intake(In);

  // Drive up to the goal
  move_rel(48, 48);

  // Turn towards the goal and drive up to it
  turn_absolute_inertial(90);
  intake(Off);
  move_rel(10, 10, 1);

  // Score the ball and back up from the goal
  dump_ball(8, 100);

  // Ensure everything is turned off
  rollers(Off);
  intake(Off);
  unlock();
  drive(0, 0);
}

void auton() {
  move_rel(0, 0);
  rollers(BottomOnly);
  wait(0.2, sec);
  rollers(Off);
  intake(In);
  wait(0.3, sec);
  intake(Off);
  
  //intake(In);
  move_rel(12, 5);
  rollers(OutTop);
  wait(1, sec);
  intake(In);
  rollers(BottomOnly);
  move_rel(-60.5, -60.5);
  rollers(Off);
  turn_absolute_inertial(90);
  task([]() {
    rollers(OutFront);
    wait(0.25, sec);
    rollers(Off);
    return 0;
  });
  intake(Off);
  move_rel(20, 20);

  task([]() {
    intake(In);
    wait(0.25, sec);
    intake(Out);
    return 0;
  });
  rollers(OutTop);
  wait(1, sec);
  intake(Off);
  rollers(Off);

  move_rel(-14, -14);
  turn_absolute_inertial(180);
  move_rel(42, 42);
  turn_absolute_inertial(135);
  intake(In);
  move_rel(26, 26);

  intake(Out);
  rollers(OutTop);
  wait(1, sec);
  intake(Off);
  rollers(Off);

  /*
  // Initialization step
  move_rel(0, 0);

  // Skip auton selection and just run skills
  if (is_skills()) {
    skills2();
    return;
  }

  // Calculate the program choice
  int prog0 = Prog0.pressing() ? 0xA0 : 0x00;
  int prog1 = Prog1.pressing() ? 0x0B : 0x00;
  int auton_choice = prog0 | prog1;

  bool reverse_turns = ReverseTurns.pressing();

  switch (auton_choice) {
    case 0x00:
      // Start orientation: face center line
      // Start position: back to middle home-row goal; aimed like skills
      // No turn reversal: gets right side
      match_auton_2(reverse_turns);
      break;

    case 0xA0:
      // Start orientation: facing center line
      // Start position: back on wall; ball in lower rollers; ramp ~1.5 in from tile junction
      // No turn reversal: gets goal to the LEFT
      match_auton_1(reverse_turns);
      break;

    case 0x0B:
      // Start orientation: facing center line
      // Start position: back on wall; ball in lower rollers; ramp ~1.5 in from tile junction
      // No turn reversal: gets left side
      match_auton_2_corner_middle(reverse_turns);
      break;

    case 0xAB:
      // Start orientation: facing center line
      // Start position: back to middle home-row goal; aimed to the right
      match_auton_home_row();
      break;
  }
  */
}

void usercontrol() {
  // Make sure the PIDs are off
  unlock();

  // Whether or not the left/right side of the base needs to be stopped
  bool stop_left = true;
  bool stop_right = true;
  // The number of loops we've run
  long ticks = 0;

  while (true) {
    // Get the left and right base speeds from the controller
    double left_speed = Controller1.Axis3.position();
    double right_speed = Controller1.Axis2.position();

    // If the input speed is below our threshold, stop the motors
    if (left_speed < 5 && left_speed > -5) {
      // This condition only calls the stop instruction once
      if (stop_left) {
        BaseLeftRear.stop(coast);
        BaseLeftFront.stop(coast);
        stop_left = false;
      }
    }
    // Otherwise spin the motors with the input 
    else {
      spin(&BaseLeftRear, left_speed);
      spin(&BaseLeftFront, left_speed);
      stop_left = true;
    }

    // This is equivalent to the code above
    if (right_speed < 5 && right_speed > -5) {
      if (stop_right) {
        BaseRightRear.stop(coast);
        BaseRightFront.stop(coast);
        stop_right = false;
      }
    } else {
      spin(&BaseRightRear, right_speed);
      spin(&BaseRightFront, right_speed);
      stop_right = true;
    }

    // Get the values for the right front buttons
    bool r1_pressing = Controller1.ButtonR1.pressing();
    bool r2_pressing = Controller1.ButtonR2.pressing();
    // Get the values for the left front buttons
    bool l1_pressing = Controller1.ButtonL1.pressing();
    bool l2_pressing = Controller1.ButtonL2.pressing();

    // The action our intake will perform
    IntakeRollerAction intake_action = Off;
    double intake_veloc = 100;
    // The action our rollers will perform
    IntakeRollerAction roller_action = Off;

    // If L1 is pressed, outtake
    if (l1_pressing && !l2_pressing) {
      intake_action = Out;
      intake_veloc = 50;
    }
    // If L2 is pressed, intake
    else if (!l1_pressing && l2_pressing) {
      intake_action = In;
    }
    // By default the intakes are off

    // In skills, link the right button to intaking
    if ((r1_pressing || r2_pressing) && is_skills()) {
      intake_action = In;
    }

    // If R1 is pressed, throw the ball out the top
    if (r1_pressing) {
      roller_action = OutTop;
    }
    // If R2 is pressed, discard the ball out the back
    else if (r2_pressing) {
      roller_action = OutFront;
    }

    // Start intake movement
    intake(intake_action, intake_veloc);

    // We use voltage for the upper roller to get slightly better performance,
    // but only for scoring out the top.
    if (roller_action == OutTop) {
      UpperRoller.spin(directionType::fwd, 12, volt);
      LowerRoller.spin(directionType::fwd, 100, pct);
    }
    // If we're not scoring out the top, then use the default function
    else {
      rollers(roller_action);
    }

    // Increase the tick count
    ticks += 1;
    wait(20.0, msec);
  }
}

competition Competition;

int main() {
  // Initialize vex's internal components
  vexcodeInit();

  // Calibrate the inertial sensor, and wait for it to finish
  Inertial.calibrate();
  while(Inertial.isCalibrating()) {
    wait(100, msec);
  }

  // Print to the screen when we're done calibrating
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Done Calibrating");

  // Initialize our PIDs and rotation tracking thread
  initialize();

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  while(true) {
    wait(50, msec);
  }
}
