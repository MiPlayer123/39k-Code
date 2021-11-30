using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor BaseLeftRear;
extern motor BaseLeftFront;
extern motor BaseRightRear;
extern motor BaseRightFront;
extern motor RearMogo;
extern motor Bar;
extern motor Claw;
extern bumper Skills;
extern inertial Inertial;
extern bumper Red;
extern bumper Blue;
extern controller Controller2;
extern bumper leftRush;
extern bumper rightRush;
extern motor Intake;
extern rotation MogoRot;
extern rotation BarRot;
extern rotation SOdom;
extern rotation ROdom;
extern rotation LOdom;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );