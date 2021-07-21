using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor BaseLeftRear;
extern motor BaseLeftFront;
extern motor BaseRightRear;
extern motor BaseRightFront;
extern motor UpperRoller;
extern motor LowerRoller;
extern motor LeftIntake;
extern motor RightIntake;
extern bumper Prog0;
extern bumper ReverseTurns;
extern bumper Prog1;
extern inertial Inertial;
extern bumper Skills;
extern bumper IsRed;
extern optical BallSensor;
extern bumper CycleTrigger;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );