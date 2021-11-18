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
// BaseRightRear        motor         11              
// BaseRightFront       motor         19              
// RearMogo             motor         14              
// Bar                  motor         9               
// Claw                 motor         17              
// Skills               bumper        H               
// Inertial             inertial      15              
// Red                  bumper        A               
// Blue                 bumper        B               
// Controller2          controller                    
// leftRush             bumper        C               
// rightRush            bumper        D               
// Intake               motor         4               
// MogoRot              rotation      16              
// BarRot               rotation      20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "vex_controller.h"
#include "kalman.h"
#include "control.h"
#include "chasis.h"

using namespace vex;

void auton() {

  if(is_skills()){
    //mogoSpin(-1.28); //Lift alliance goal
    inertial_drive(-10, 20);
    //mogoSpin(1);
    turn_absolute_inertial(90);
    moveRot(.5,40);
    turn_absolute_inertial(90);
    inertial_drive(46, 50);
    closeClaw(); //Grab yellow
    setBar(0.2);
    inertial_drive(-12, 40);
    turn_absolute_inertial(228);
    setBar(1.3);
    inertial_drive(35.4, 40);
    setBar(-.3);
    openClaw(); //Score 1st yellow
    inertial_drive(-5.5, 30);
    turn_absolute_inertial(144);
    //turn_absolute_inertial(180);
    inertial_drive(30, 50);
    setBar(-1.2);
    inertial_drive(26, 30);
    closeClaw(); //grab 2nd yellow
    setBar(.3);
    turn_absolute_inertial(270);
    inertial_drive(15, 50);
    //spinBar(60);
    turn_absolute_inertial(303);
    //stopBar();
    setBar(.5);
    inertial_drive(28, 40);
    setBar(-.2);
    openClaw(); //Drop 2nd yellow
    inertial_drive(-5, 40);
    setBar(-1.2);
    turn_absolute_inertial(182);
    //stopBar();
    inertial_drive(21, 40);
    closeClaw(); //grab 2nd alliance
    //inertial_drive(-5, 30);
    turn_absolute_inertial(90);
    inertial_drive(75, 70);
    turn_absolute_inertial(270);
    //mogoSpin(-1.28);
    inertial_drive(5, 30);
    turn_absolute_inertial(225);
    inertial_drive(-10, 30);  
    //mogoSpin(1.28);
    inertial_drive(10, 30);
    turn_absolute_inertial(315);
    inertial_drive(60, 70);

    /*
    mogoSpin(1.28);//grab other alliance rear
    inertial_drive(25, 30);
    turn_absolute_inertial(0);
    inertial_drive(8, 30);
    turn_absolute_inertial(90);
    //mikul insert your scoring thing //score 2nd allance
    turn_absolute_inertial(0);
    inertial_drive(48, 40);
    closeClaw(); //grab other other alliance goal
    inertial_drive(-5, 30);
    turn_absolute_inertial(315);
    inertial_drive(60, 70); //Push tall goal
    turn_absolute_inertial(90);
    mogoSpin(-1.28);
    inertial_drive(10, 30);
    mogoSpin(1.28); //drop other alliance goal
    turn_absolute_inertial(0);
    inertial_drive(15, 30);
    turn_absolute_inertial(270);
    */
    //scoring thing
    /* 2 neutrals + 2 alliance plat 
       1 neutral + 2 alliance homezone 
        220 auton run 
    */
  } 
  
  else if (Red.pressing()){
    //Right auto
    turn_absolute_inertial(45);
    setBar(.6);
    inertial_drive(15, 50);
    clawSpinT(-.7); //Drop rings
    inertial_drive(-5, 50);
    setBar(-.6);
    inertial_drive(10, 50);
    inertial_drive(-1.8, 30);
    turn_absolute_inertial(-44.2);
    inertial_drive(58, 80); //Grab Goal
    moveRot(.01, 10);
    inertial_drive(5, 30);
    closeClaw();
    inertial_drive(-60, 70);
    turn_absolute_inertial(135);
    inertial_drive(5, 30);
    setBar(.5);
    openClaw();

    /* WIP
    Claw.spin(fwd,20,pct);
    inertial_drive(48, 98.5);
    Claw.stop(hold);
    clawSpinT(.3);
    inertial_drive(-33, 50);
    setBar(.2);
    turn_rel_inertial(-75);
    inertial_drive(-1.5, 20);
    mogoSpin(-.8);
    moveRot(.1,30);
    RearMogo.spin(fwd,-5,pct);
    inertial_drive(3, 10);
    */
    /*old auto
    moveRot(2, 30);
    setBar(.6);
    moveRot(2.5,30);
    clawSpinT(-.6);
    moveRot(-1,30);
    setBar(-.6);
    moveRot(5,30);
    moveRot(-3,30);
    */
  } 
  
  else if (Blue.pressing()){
    //Left auto
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
    /*
    turn_absolute_inertial(0);
    Claw.spin(fwd,25,pct);
    inertial_drive(48, 98.5);
    Claw.stop(hold);
    clawSpinT(.3);
    inertial_drive(-37, 50);
    */
    //Old Auto
    /*
    moveRot(2, 30);
    setBar(.6);
    moveRot(2.5,30);
    clawSpinT(-.6);
    moveRot(-1,30);
    setBar(-.6)*/
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
    //Claw.spin(fwd,25,pct);
    inertial_drive(48, 98.5);
    //Claw.stop(hold);
    closeClaw();
    inertial_drive(-37, 50);
    /*
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
    */
  }
  else{
    //turn_absolute_inertial(90);
    /*
    Claw.spin(fwd,25,pct);
    inertial_drive(48, 98.5);
    Claw.stop(hold);
    clawSpinT(.3);
    inertial_drive(-45, 50);
    */
    //inertial_drive(72, 60);
    closeClaw();
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

 

    // If L1 is pressed, claw
    if (l1_pressing) {
      Claw.spin(fwd,100,pct);
    }
    // If L2 is pressed, claw
    else if (l2_pressing) {
        Claw.spin(reverse,100,pct);
    }
    else{
      Claw.stop(brake);
    }
    // By default the intakes are off

    if (r1_pressing) { //&& BarRot.position(deg)>=0
      Bar.spin(fwd,100,pct);
    }
    else if (r2_pressing ) { //&& BarRot.position(deg)<=134
      Bar.spin(reverse,100,pct);
    }

    else {
      Bar.stop(hold);
    }

    if (Controller1.ButtonUp.pressing()) {
      RearMogo.spin(fwd,60,pct);
    }
    else if (Controller1.ButtonDown.pressing()) {
      RearMogo.spin(reverse,60,pct);
    } else if(Controller1.ButtonLeft.pressing()){
      //Drop mogo down
      mogoPos(3, false);
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

  Claw.setPosition(0, degrees);
  Bar.setPosition(0, deg);

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  while(true) {
    wait(50, msec);
  }
}