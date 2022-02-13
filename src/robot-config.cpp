#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor BaseLeftRear = motor(PORT14, ratio6_1, false);
motor BaseLeftFront = motor(PORT13, ratio6_1, false);
motor BaseRightRear = motor(PORT20, ratio6_1, true);
motor BaseRightFront = motor(PORT3, ratio6_1, true);
motor RearMogo = motor(PORT12, ratio36_1, true);
motor Bar = motor(PORT1, ratio36_1, false);
motor Claw = motor(PORT5, ratio36_1, true);
bumper Skills = bumper(Brain.ThreeWirePort.H);
inertial Inertial = inertial(PORT19);
bumper Right = bumper(Brain.ThreeWirePort.A);
bumper Left = bumper(Brain.ThreeWirePort.B);
controller Controller2 = controller(partner);
bumper leftRush = bumper(Brain.ThreeWirePort.C);
bumper rightRush = bumper(Brain.ThreeWirePort.D);
motor Intake = motor(PORT18, ratio6_1, false);
rotation MogoRot = rotation(PORT11, false);
rotation BarRot = rotation(PORT15, false);
rotation ROdom = rotation(PORT7, false);
rotation LOdom = rotation(PORT17, false);
gps GPS = gps(PORT16, 152.40, 50.80, mm, 180);
rotation SOdom = rotation(PORT10, false);
digital_out Pn = digital_out(Brain.ThreeWirePort.F);
limit LimitSwitch = limit(Brain.ThreeWirePort.G);
limit LimitSwitch2 = limit(Brain.ThreeWirePort.E);
distance Distance = distance(PORT6);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}