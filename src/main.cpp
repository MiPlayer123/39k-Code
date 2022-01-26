// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// BaseLeftRear         motor         14              
// BaseLeftFront        motor         13              
// BaseRightRear        motor         20              
// BaseRightFront       motor         3               
// RearMogo             motor         12              
// Bar                  motor         1               
// Claw                 motor         4               
// Skills               bumper        H               
// Inertial             inertial      19              
// Red                  bumper        A               
// Blue                 bumper        B               
// Controller2          controller                    
// leftRush             bumper        C               
// rightRush            bumper        D               
// Intake               motor         18              
// MogoRot              rotation      11              
// BarRot               rotation      15              
// ROdom                rotation      7               
// LOdom                rotation      17              
// GPS                  gps           16              
// Carry                bumper        E               
// SOdom                rotation      10              
// Pn                   digital_out   F               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Mikul & Lily                                              */
/*    Created:      Thu Sep 10 2020                                           */
/*    Description:  39K project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "vex_controller.h"
#include "kalman.h"
#include "control.h"
#include "chasis.h"
#include "DrawField.h"
#include "odometry.h"

using namespace vex;

task odometryTask;
task drawFieldTask;
task mogoHeightTask;

void auton() {

  task odometryTask(positionTracking);
  task drawFieldTask(drawField);

  if(is_skills()){
    mogoPos(3, false);
    inertial_drive(-8, 30);
    mogoPos(2, false); //Grab first red
    turn_absolute_inertial(60);
    inertial_drive(30.5, 70);
    turn_absolute_inertial(125);
    inertial_drive(24, 70);
    closeClaw(); //grab yellow
    setBar(105);
    spinIntake();
    inertial_drive(51.5, 60);
    turn_absolute_inertial(95);
    inertial_drive(7, 30);
    stopIntake();
    setBar(75);
    vex::task::sleep(250);
    openClaw(); //Score yellow
    turn_absolute_inertial(0);
    setBar(4);
    inertial_drive(11, 75);
    turn_absolute_inertial(0);
    mogoPos(3, false); //Drop red
    inertial_drive(32, 70);
    closeClaw(); //Grab blue
    moveRot(-1, 30);
    setBar(20);
    inertial_drive(-11, 60);
    turn_absolute_inertial(45); //37
    inertial_drive(-110, 70);
    //turn_absolute_inertial(80); //push Yellow tall
    //inertial_drive(-23, 70);
    inertial_drive(12, 50); //3
    turn_absolute_inertial(0);
    mogoRotation(.42);
    setBar(95);
    inertial_drive(24, 80);
    turn_absolute_inertial(-90);
    moveRot(3.7, 50);
    openClaw(); //Score Blue
    turn_absolute_inertial(-0);
    mogoPos(3, false);
    vex::task::sleep(250);
    inertial_drive(-24, 70);
    turn_absolute_inertial(-0);
    setBar(4);
    inertial_drive(-24, 70);
    turn_absolute_inertial(-0);
    //inertial_drive(-1, 70);
    mogoPos(2, false); //Grab 2nd Blue
    inertial_drive(14, 70);
    turn_absolute_inertial(90);
    //turn_absolute_inertial(63);
    inertial_drive(24, 70);
    closeClaw(); //Grab 3rd Yellow
    setBar(95);
    turn_absolute_inertial(63);
    spinIntake();
    inertial_drive(50, 80);
    openClaw(); //Score 3rd yellow
    startBar(-70);
    inertial_drive(-25.3, 70);
    turn_absolute_inertial(90);
    stopBar();
    setBar(4);
    stopIntake();
    inertial_drive(34, 80);
    closeClaw(); //Grab 2nd red
    inertial_drive(-30, 80);
    setBar(20);
    turn_absolute_inertial(287);
    mogoPos(3, false); //Drop 2nd blue
    setBar(95);
    inertial_drive(67, 90);
    openClaw(); //Score red (220+)
    moveRot(-3, 100);
  } 
  
  else if (Red.pressing()){
    //Right auto
    inertial_drive(45, 99);
    closeClaw();
    inertial_drive(-15, 70);
    clawSpinT(.1);
    setBar(15);
    turn_absolute_inertial(0);
    inertial_drive(-18.5, 70);
    turn_absolute_inertial(-90);
    mogoPos(3, false);
    inertial_drive(-10, 40);
    mogoPos(2, false);
    spinIntake();
    vex::task::sleep(2000);
    stopIntake();
  } 
  
  else if (Blue.pressing()){
    //Left auto
  } 
  else if(leftRush.pressing()){
    /*
    //Claw.spin(fwd,20,pct);
    moveRot(0.5,10);
    inertial_drive(48,98);
    //Claw.stop(brake);
    clawSpinT(.3);
    inertial_drive(-40, 90);
    */

    turn_absolute_inertial(5);
    moveRot(10,250);
    spin(&BaseRightRear, 40);
    spin(&BaseRightFront, 40);
    spin(&BaseLeftRear, 40);
    spin(&BaseLeftFront, 40);
    Claw.spin(fwd,80,pct);
    vex::task::sleep(700);
    BaseLeftRear.stop(brake);
    BaseLeftFront.stop(brake);
    BaseRightRear.stop(brake);
    BaseRightFront.stop(brake);
    Claw.stop(hold);
    moveRot(-10,70);
  }
  else if (rightRush.pressing()) {
    //Right rush

  } 
  else if(Carry.pressing()){
    //WP Mid
    setBar(40);
    moveRot(1, 30);
    openClaw();
    moveRot(-1, 30);
    turn_absolute_inertial(-90);
    inertial_drive(18, 50);
    turn_absolute_inertial(-180);
    setBar(3);
    mogoPos(3, false);
    inertial_drive(-96, 70);
    mogoPos(2, false);
    spinIntake();
    inertial_drive(25, 30);
    stopIntake();
    turn_absolute_inertial(45);
    /*
    inertial_drive(30, 50);
    closeClaw();
    inertial_drive(-15, 50);
    */
  }
  else{
    startBar(70);
    mogoRotation(.42);
    stopBar();
    //turn_absolute_inertial(90);
  }
} 

void usercontrol() {
  
  // Whether or not the left/right side of the base needs to be stopped
  bool stop_left = true;
  bool stop_right = true;
  // The number of loops we've run
  long ticks = 0;

  //Intake stat
  //bool intakeStat = false;

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
    if (right_speed < 10 && right_speed > -10) {
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

 
    //Nathan
    // If L1 is pressed, claw
    if (l1_pressing) {
      Claw.spin(fwd,100,pct);
    }
    // If L2 is pressed, claw
    else if (l2_pressing) {
        Claw.spin(reverse,100,pct);
    }
    else{
      Claw.stop(hold);
    }
    // By default the intakes are off

    if (r1_pressing) { //&& BarRot.position(deg)>=0
      Bar.spin(fwd,100,pct);
    }
    else if (r2_pressing) { //&& BarRot.position(deg)<=134
      Bar.spin(reverse,100,pct);
    }

    else {
      Bar.stop(hold);
    }
    

    /*//Nate
    // If L1 is pressed, claw
    if (r1_pressing) {
      Claw.spin(fwd,100,pct);
    }
    // If L2 is pressed, claw
    else if (l1_pressing) {
        Claw.spin(reverse,100,pct);
    }
    else{
      Claw.stop(hold);
    }
    // By default the intakes are off

    if (l2_pressing) { //&& BarRot.position(deg)>=0
      Bar.spin(fwd,100,pct);
    }
    else if (r2_pressing) { //&& BarRot.position(deg)<=134
      Bar.spin(reverse,100,pct);
    }

    else {
      Bar.stop(hold);
    }
    */

    bool lock= false;

    if (Controller1.ButtonUp.pressing()) { // && (MogoRot.position(deg)<194 || MogoRot.position(deg)>300)
      RearMogo.spin(fwd,100,pct);
    }
    else if (Controller1.ButtonDown.pressing()) { // && MogoRot.position(deg)>0
      RearMogo.spin(reverse,100,pct);
    } else if(Controller1.ButtonLeft.pressing()){
      //Drop mogo down
      lock=!lock;
      if(lock){
        BaseLeftFront.setBrake(hold);
        BaseLeftRear.setBrake(hold);
        BaseRightFront.setBrake(hold);
        BaseRightRear.setBrake(hold);
      }else{
        brake_unchecked();
      }
    } else if (Controller1.ButtonRight.pressing()){
      //Mogo to ring height
      mogoPos(2, false);
    }
    else {
      RearMogo.stop(hold);
    }

    if(Controller1.ButtonA.pressing()){
      Intake.spin(fwd, 100, pct);
    } 
    else if(Controller1.ButtonY.pressing()){
      Intake.stop(coast);
    }

    if(Controller1.ButtonX.pressing()){
      Pn.set(true);
    } else if (Controller1.ButtonB.pressing()) {
      Pn.set(false);
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

  //Set encoders to 0
  ROdom.setPosition(0, deg);
  LOdom.setPosition(0, deg);
  SOdom.setPosition(0, deg);

  // Calibrate the inertial sensor, and wait for it to finish
  Inertial.calibrate();
  while(Inertial.isCalibrating()) {
    wait(100, msec);
  }

  //imagine the field is a unit circle

  //The starting x and y coordinates of the bot (INCHES)
  //These distances are relative to some point (0,0) on the field
  /*
  if(is_skills()){
    THETA_START = M_PI/2; 
    X_START = 56.5; //19.1
    Y_START = 8.5; //8.5
  } else{
    THETA_START = M_PI; 
    X_START = 0; //19.1
    Y_START = 0; //8.5
  }
  THETA_START = M_PI; 
  X_START = 0; //19.1
  Y_START = 0; //8.5
  */

  // Print to the screen when we're done calibrating
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Done Calibrating");

  // Initialize our PIDs and rotation tracking thread
  initialize();
  //task mogoHeightTask(mogoHeight);

  Claw.setPosition(0, degrees);
  Bar.setPosition(0, deg);
  BaseLeftRear.setPosition(0, deg);
  BaseRightRear.setPosition(0, deg);

  GPS.setOrigin(-1800, -1800);

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  while(true) {
    wait(50, msec);
  }
}