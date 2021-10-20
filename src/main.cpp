
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
// FrontMogo            motor         20              
// RearMogo             motor         14              
// Bar                  motor         13              
// Claw                 motor         10              
// Skills               bumper        H               
// Inertial             inertial      15              
// Red                  bumper        A               
// Blue                 bumper        B               
// Controller2          controller                    
// Bar2                 motor         16              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "vex_controller.h"
#include "kalman.h"
#include "control.h"
#include "chasis.h"

using namespace vex;

void auton() {

  if(is_skills()){
    //Skills
  } 
  
  else if (Red.pressing()){
    //Red auto
    moveRot(2, 30);
    setBar(.6);
    moveRot(2.5,30);
    clawSpinT(-.6);
    moveRot(-1,30);
    setBar(-.6);
    moveRot(5,30);
    moveRot(-3,30);
  } 
  
  else if (Blue.pressing()){
    //Blue auto
  } else{
    //turn_absolute_inertial(45);
    mogoSpin(-1.34);
    moveRot(-2.5, 30);
    mogoSpin(1);
    turn_absolute_inertial(90);
    moveRot(.5,40);
    turn_absolute_inertial(90);
    moveRot(10, 40);
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
    moveRot(4,40);
    setBar(.2);
    turn_absolute_inertial(120);
    moveRot(2,40);
    setBar(1.5);
    moveRot(7,40);
    turn_absolute_inertial(115);
    setBar(-.3);
    clawSpinT(-.7);
    moveRot(-.5,30); //goal on platform
    moveRot(-3.2,30);
    turn_absolute_inertial(225);
    setBar(-1);
    mogoSpin(-1);
    moveRot(24,40);
    moveRot(-2.9,30);
    turn_absolute_inertial(90);
    moveRot(15,40);
  }
} 

void usercontrol() {
  
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

 

    // If L1 is pressed, claw
    if (l1_pressing) {
      Claw.spin(fwd,100,pct);
    }
    // If L2 is pressed, claw
    else if (l2_pressing) {
        Claw.spin(reverse,100,pct);
    }
    else{
      Claw.stop( brake);
    }
    // By default the intakes are off

    // In skills, link the right button to intaking
    if (r1_pressing) {
      Bar.spin(fwd,100,pct);
      Bar2.spin(fwd,100,pct);
    }
    else if (r2_pressing) {
      Bar.spin(reverse,100,pct);
      Bar2.spin(reverse,100,pct);
    }

    else {
      Bar.stop(hold);
      Bar2.stop(hold);
    }

    if (Controller2.installed()){
          // If L1 is pressed, claw
      if (Controller2.ButtonR1.pressing()) {
        FrontMogo.spin(reverse,100,pct);
      }
      // If L2 is pressed, claw
      else if (Controller2.ButtonR2.pressing()) {
          FrontMogo.spin(fwd,100,pct);
      }
      else{
        FrontMogo.stop( hold);
      }
      // By default the intakes are off

      // In skills, link the right button to intaking
      if (Controller2.ButtonL1.pressing()) {
        RearMogo.spin(fwd,100,pct);
      }
      else if (Controller2.ButtonL2.pressing()) {
        RearMogo.spin(reverse,100,pct);
      }

      else {
        RearMogo.stop(hold);
      }
    } else{
                // If L1 is pressed, claw
      if (Controller1.ButtonX.pressing()) {
        FrontMogo.spin(reverse,100,pct);
      }
      // If L2 is pressed, claw
      else if (Controller1.ButtonB.pressing()) {
          FrontMogo.spin(fwd,100,pct);
      }
      else{
        FrontMogo.stop( hold);
      }
      // By default the intakes are off

      // In skills, link the right button to intaking
      if (Controller1.ButtonUp.pressing()) {
        RearMogo.spin(fwd,100,pct);
      }
      else if (Controller1.ButtonDown.pressing()) {
        RearMogo.spin(reverse,100,pct);
      }

      else {
        RearMogo.stop(hold);
      }
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
