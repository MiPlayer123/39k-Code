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
// LOdom                rotation      17              
// GPS                  gps           16              
// SOdom                rotation      10              
// Pn                   digital_out   F               
// LimitSwitch          limit         G               
// LimitSwitch2         limit         E               
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

void insaneAuton() {
  Pn.set(true);
  allBaseVoltage(true, 12);
  while(!LimitSwitch.pressing() && !LimitSwitch2.pressing()){
    vex::task::sleep(5); 
  }
  clawSpinT(.2);
  allBaseVoltage(false, 12);
  Pn.set(false);
  vex::task::sleep(1*1000); 
  /* wall reset against back wall at a slower speed
  drive out, turn, wall reset against side wall
  forward until goal
  turn 180, pick up w back, rings
  */
}

void auton() {

  task odometryTask(positionTracking);
  task drawFieldTask(drawField);

  if(is_skills()){
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
    turn_absolute_inertial(120);
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
    setBar(20);
    turn_absolute_inertial(145);
    startBar(90);
    inertial_drive(40, 80);
    stopBar();
    setBar(90);
    turn_absolute_inertial(115);
    openClaw(); //Score 1st blue
    turn_absolute_inertial(220);
    spinIntake();
    inertial_drive(30, 50);
    stopIntake();
    startBar(-20);
    inertial_drive(-6, 80);
    setBar(2);
    inertial_drive(14, 40);
    closeClaw(); //2nd yellow
    setBar(25);
    turn_absolute_inertial(65);
    startBar(100);
    inertial_drive(18, 80);
    setBar(92);
    inertial_drive(20, 55);
    stopBar();
    openClaw(); //score 2nd yellow
    moveRot(-1, 80);
    turn_absolute_inertial(240);
    startBar(-30);
    inertial_drive(40, 90);
    stopBar();
    spinIntake();
    inertial_drive(30, 50);
    inertial_drive(-6, 80);
    setBar(2);
    stopIntake();
    inertial_drive(14, 40);
    closeClaw(); //Grab blue
    setBar(20);
    inertial_drive(-10, 80);
    turn_absolute_inertial(50);
    inertial_drive(60, 90);
    
    /* //22o
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
    inertial_drive(-70, 80);
    setMogo(20);
    inertial_drive(-40, 80);
    mogoPos(3, false);
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
    moveRot(-3, 100); */
  } 
  
  else if (Right.pressing()){
    //Right auto
    inertial_drive(45, 99);
    closeClaw();
    inertial_drive(-20, 90);
    clawSpinT(.1);
    setBar(20);
    turnRot(-.4, 70);
    turn_absolute_inertial(-60);
    mogoPos(3, false);
    inertial_drive(-15, 40);
    mogoPos(2, false);
    spinIntake();
    vex::task::sleep(2000);
    stopIntake();
  } 
  
  else if (Left.pressing()){
    //Left auto
  } 
  else if(leftRush.pressing()){
    turnRot(.3, 80);
    //turn_absolute_inertial(5);
    inertial_drive(47, 99);
    closeClaw();
    inertial_drive(-20, 90);
    clawSpinT(.1);
    setBar(20);
    turn_absolute_inertial(30);
    inertial_drive(-25, 50);
    mogoPos(3, false);
    turn_absolute_inertial(-50);
    inertial_drive(-16, 50);
    mogoPos(2, false);
    spinIntake();
    vex::task::sleep(2000);
    stopIntake();
  }
  else if (rightRush.pressing()) {
    //Right rush

  }
  else{
    insaneAuton();
    brake_unchecked();
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