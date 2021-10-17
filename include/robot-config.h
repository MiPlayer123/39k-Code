using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor BaseLeftRear;
extern motor BaseLeftFront;
extern motor BaseRightRear;
extern motor BaseRightFront;
extern motor FrontMogo;
extern motor RearMogo;
extern motor Bar;
extern motor Claw;
extern bumper Skills;
extern inertial Inertial;
extern bumper Red;
extern bumper Blue;
extern controller Controller2;
extern motor Bar2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );