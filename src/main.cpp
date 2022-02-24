// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// BaseLeftRear         motor         14              
// BaseLeftFront        motor         13              
// BaseRightRear        motor         20              
// BaseRightFront       motor         2               
// RearMogo             motor         17              
// Bar                  motor         1               
// Claw                 motor         5               
// Skills               bumper        H               
// Inertial             inertial      19              
// Right                bumper        A               
// Left                 bumper        B               
// Controller2          controller                    
// leftRush             bumper        C               
// rightRush            bumper        D               
// Intake               motor         18              
// MogoRot              rotation      11              
// BarRot               rotation      15              
// ROdom                rotation      7               
// LOdom                rotation      3               
// GPS                  gps           21              
// SOdom                rotation      10              
// Pn                   digital_out   G               
// Distance             distance      6               
// Distance2            distance      16              
// Elims                bumper        F               
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

void insaneAuton(float tim=1) {
  Pn.set(true);
  allBaseVoltage(true, 12);
  int dist;
  if(Elims.pressing()){
    dist=80;
  }
  else{
    dist=100;
  }
  while((Distance.value()>dist || !Distance.isObjectDetected())){ 
    vex::task::sleep(5); 
  }
  clawSpinT(.23);
  allBaseVoltage(false, 12);
  Pn.set(false);
  vex::task::sleep(tim*1000); 
}

void auton() {

  task odometryTask(positionTracking);
  task drawFieldTask(drawField);

  if(is_skills()){
    //280
    mogoPos(3, false);
    inertial_drive(-8, 30);
    mogoPos(2, false); //Grab first red
    turn_absolute_inertial(60);
    inertial_drive(30.5, 70);
    turn_absolute_inertial(127);
    spinIntake();
    inertial_drive(24, 70);
    closeClaw(); //grab yellow  
    stopIntake();
    setBar(20);
    startBar(105);
    turn_absolute_inertial(120, true);
    inertial_drive(30, 85);
    stopBar();
    mogoPos(3, false); //Drop blue
    inertial_drive(20, 80);
    setBar(74);
    openClaw(); //Score yellow
    moveRot(-1.7, 80);
    turn_absolute_inertial(183);
    startBar(-35);
    inertial_drive(-36, 70);
    vex::task::sleep(100);
    mogoPos(2, false); //Grab red
    stopBar();
    turn_absolute_inertial(225);
    setBar(2);
    inertial_drive(12, 90);
    inertial_drive(12, 50);
    closeClaw(); //Grab first red
    setBar(25);
    turn_absolute_inertial(145, true);
    startBar(90);
    inertial_drive(40, 80);
    stopBar();
    setBar(90);
    turn_absolute_inertial(110, true);
    openClaw(); //Score 1st blue
    turn_absolute_inertial(180);
    startBar(-20);
    spinIntake();
    inertial_drive(32, 95);
    inertial_drive(-10, 80);
    turn_absolute_inertial(240);
    stopBar();
    stopIntake();
    setBar(2);
    moveRot(1.5, 60);
    driveToGoal(); //2nd yellow
    setBar(25);
    turn_absolute_inertial(55, true);
    startBar(100);
    inertial_drive(18, 80);
    setBar(92);
    inertial_drive(20, 55);
    stopBar();
    openClaw(); //score 2nd yellow
    moveRot(-1, 80);
    turn_absolute_inertial(240);
    startBar(-30);
    inertial_drive(30, 90);
    stopBar();
    turn_absolute_inertial(270);
    spinIntake();
    inertial_drive(35, 80);
    turn_absolute_inertial(180);
    setBar(2);
    stopIntake();
    inertial_drive(6, 45);
    driveToGoal();//Grab blue
    inertial_drive(-10, 80);
    setBar(30);
    turn_absolute_inertial(65, true);
    spinIntake();
    startBar(50);
    inertial_drive(85, 90);
    setBar(92);
    openClaw(); //Drop blue
    stopIntake();
    startBar(-50);
    inertial_drive(-13, 80);
    turn_absolute_inertial(-45);
    setBar(2);
    inertial_drive(13, 70); //15
    driveToGoal(6); //grab tall yellow
    setBar(20);
    turn_absolute_inertial(-90, true);
    startBar(100);
    spinIntake();
    inertial_drive(30, 85);
    setBar(70);
    inertial_drive(10, 60);
    openClaw(); //Score tall yellow
    stopIntake();
    
    /* //NEEDS TESTING
    moveRot(-1, 90);
    startBar(-20);
    turn_absolute_inertial(110);
    inertial_drive(50, 90);
    turn_absolute_inertial(30);
    setBar(2);
    stopIntake();
    inertial_drive(10, 60);
    driveToGoal(); //Grab 2nd red
    */
  } 
  
  else if (Right.pressing()){
    //Right auto rush WP
    insaneAuton();
    insaneAuton(0);
    brake_unchecked();
    voltageDist(920);
    setBar(20);
    inertial_drive(29.5, 70, true);
    turn_absolute_inertial(-90);
    inertial_drive(10, 80);
    mogoPos(3, false);
    inertial_drive(-23, 60);
    mogoPos(2,false); //Pick goal
    setBar(50);
    spinIntake();
    turn_absolute_inertial(0, true);
    inertial_drive(40, 50);
    inertial_drive(-45, 90);
    stopIntake();
  } 
  
  else if (Left.pressing()){
    //Left double WP
    inertial_drive(16.5, 80);
    turn_absolute_inertial(90);
    inertial_drive(75, 80);
    turn_absolute_inertial(-90);
    mogoPos(3, false);
    inertial_drive(-12, 50);
    mogoPos(2, false);
    spinIntake();
    vex::task::sleep(750);
    turn_absolute_inertial(-55);
    stopIntake();
    inertial_drive(47, 60);
    closeClaw();
    inertial_drive(-40, 90);
    turnRot(2, 80);
    inertial_drive(-10, 90);
    
  } 
  else if(leftRush.pressing()){
    //Left double rush WP
    insaneAuton(0);
    brake_unchecked();
    voltageDist(1150);
    inertial_drive(38, 50,true);
    setBar(25);
    moveRot(.45, 80);
    turn_absolute_inertial(-70);
    inertial_drive(-15, 80);
    moveRot(2, 80);
    turn_absolute_inertial(-120);
    mogoPos(3, false);
    inertial_drive(-29, 60); //Distance
    //inertial_drive(1.2, 60, true); //Sensor
    setMogo(27);
    inertial_drive(50, 95);
  }
  else if (rightRush.pressing()) {
    //Right double rush WP
    //Currently not needed
  }
  else{
   insaneAuton(0); //Single rush
    brake_unchecked();
    voltageDist(920);
    turn_absolute_inertial(0);
    inertial_drive(24, 50,true);

    //turn_absolute_inertial(90,true);
    
  }
} 

bool lock= false;

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
        if(lock){
        BaseLeftRear.stop(hold);
        BaseLeftFront.stop(hold);
        }else{
          BaseLeftRear.stop(coast);
          BaseLeftFront.stop(coast);
        }
        // BaseLeftRear.stop(coast);
        // BaseLeftFront.stop(coast);
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
        if(lock){
        BaseRightRear.stop(hold);
        BaseRightFront.stop(hold);
      }else{
        BaseRightRear.stop(coast);
        BaseRightFront.stop(coast);
      }
      // BaseRightRear.stop(coast);
      // BaseRightFront.stop(coast);
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

    if (Controller1.ButtonUp.pressing()) { // && (MogoRot.position(deg)<194 || MogoRot.position(deg)>300)
      RearMogo.spin(fwd,100,pct);
    }
    else if (Controller1.ButtonDown.pressing()) { // && MogoRot.position(deg)>0
      RearMogo.spin(reverse,100,pct);
    } else if(Controller1.ButtonLeft.pressing()){
      //Toggle lock
      lock=!lock;
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