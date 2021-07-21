#ifndef __SUBSYS_H
#define __SUBSYS_H

#include "vex.h"
#include "util.h"

// An action that the intakes or rollers can take
enum IntakeRollerAction {
  // Intakes take the ball in
  In,
  // Intakes discard the ball out front
  Out,
  // Rollers score the ball out the top
  OutTop,
  // Rollers discard the ball out the back
  OutBack,
  // Rollers roll the ball down into the intakes
  OutFront,
  // Run only the top roller in the intakes
  TopOnly,
  // Run only the bottom roller in the intakes
  BottomOnly,
  // Turn the intakes or rollers off
  Off
};

// A method for discarding a ball
enum BallDiscardMethod {
  // Throw balls that aren't our color out the back
  DiscardOutBack,
  // Cycle the goal until it's flipped to our color
  CycleUntilFlipped,
  // Don't discard balls of the opponent's color
  NoDiscard
};

// Power the base with the given left and right percentages
void drive(double left, double right, vex::brakeType brake_type=brake);

// Have the base oscillate between moving forward and backward
task wiggle();

// Have the intakes perform the specified action
void intake(IntakeRollerAction action, double veloc=100);

// Have the rollers perform the specified action
void rollers(IntakeRollerAction action, double veloc=100);

// Release our hood
task release_flap();

// Cycle the given number of balls regardless of color
void cycle_n(int num_balls);

// Intake a ball into the middle of the rollers
void intake_mid();

// Outtake one ball through the rollers and intakes
void outtake_one();

// Intake one ball into the lower rollers
bool intake_low(double max_time=0);

// Lower the balls in the rollers so they can be scored more effectively
void lower_rollers(double time=0.5);

// A state to track the progress in cycling
class CycleState {
  private:
  Ball ball_in_lower_rollers;
  IntakeRollerAction roller_state;
  double state_duration;
  double time_since_last_state;

  public:
  int num_scored;
  BallDiscardMethod discard_method;

  CycleState();

  void update(double time_elapsed);

  void reset();
};

extern CycleState cycle_state;

#endif