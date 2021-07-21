#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor BaseLeftRear = motor(PORT1, ratio6_1, false);
motor BaseLeftFront = motor(PORT2, ratio6_1, true);
motor BaseRightRear = motor(PORT21, ratio6_1, true);
motor BaseRightFront = motor(PORT9, ratio6_1, false);
motor UpperRoller = motor(PORT7, ratio6_1, false);
motor LowerRoller = motor(PORT4, ratio6_1, true);
motor LeftIntake = motor(PORT5, ratio6_1, false);
motor RightIntake = motor(PORT10, ratio6_1, true);
bumper Prog0 = bumper(Brain.ThreeWirePort.A);
bumper ReverseTurns = bumper(Brain.ThreeWirePort.G);
bumper Prog1 = bumper(Brain.ThreeWirePort.B);
inertial Inertial = inertial(PORT15);
bumper Skills = bumper(Brain.ThreeWirePort.H);
bumper IsRed = bumper(Brain.ThreeWirePort.F);
optical BallSensor = optical(PORT17);
bumper CycleTrigger = bumper(Brain.ThreeWirePort.E);

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