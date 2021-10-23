#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor BaseLeftRear = motor(PORT1, ratio6_1, false);
motor BaseLeftFront = motor(PORT2, ratio6_1, false);
motor BaseRightRear = motor(PORT11, ratio6_1, true);
motor BaseRightFront = motor(PORT19, ratio6_1, true);
motor FrontMogo = motor(PORT20, ratio36_1, true);
motor RearMogo = motor(PORT14, ratio36_1, false);
motor Bar = motor(PORT13, ratio36_1, true);
motor Claw = motor(PORT10, ratio18_1, false);
bumper Skills = bumper(Brain.ThreeWirePort.H);
inertial Inertial = inertial(PORT15);
bumper Red = bumper(Brain.ThreeWirePort.A);
bumper Blue = bumper(Brain.ThreeWirePort.B);
controller Controller2 = controller(partner);
motor Bar2 = motor(PORT16, ratio36_1, false);
bumper leftRush = bumper(Brain.ThreeWirePort.C);
bumper rightRush = bumper(Brain.ThreeWirePort.D);

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