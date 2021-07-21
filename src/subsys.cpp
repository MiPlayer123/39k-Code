#include "subsys.h"
#include "util.h"

#define MIN_STATE_GRACE_PERIOD 0.1 // seconds
#define SCORE_TIME 0.75 // seconds

CycleState::CycleState() {
  ball_in_lower_rollers = None;
  roller_state = Off;
  state_duration = 0;
  time_since_last_state = 0;
  num_scored = 0;
  discard_method = DiscardOutBack;
}

void CycleState::update(double time_elapsed) {
  time_since_last_state += time_elapsed;

  // If a ball is not transitioning to the upper roller, no need to change the state
  if (!CycleTrigger.pressing()) {
    // Make sure we read the incoming ball accurately
    if (ball_in_lower_rollers == None) {
      ball_in_lower_rollers = read_optical(&BallSensor);
    }
    return;
  }

  Ball tc = team_color(); // Team color
  Ball oc = invert_color(tc); // Opposing color
  double state_time_remaining = state_duration - time_since_last_state;

  // Find a new state
  if (state_time_remaining <= 0) {
    // If the ball in the rollers matches the team color, then score it
    if (ball_in_lower_rollers == tc) {
      roller_state = OutTop;
      state_duration = SCORE_TIME;
      num_scored += 1;
    }
    // If the color does not match, decide how to handle it
    else if (ball_in_lower_rollers == oc) {
      switch (discard_method) {
        // Discard the ball out the back of the robot
        case DiscardOutBack:
          roller_state = OutBack;
          state_duration = MIN_STATE_GRACE_PERIOD;
          break;

        // If we haven't scored our color yet, it doesn't matter if we score the
        // opposing color.
        case CycleUntilFlipped:
          if (num_scored == 0) {
            roller_state = OutTop;
            state_duration = SCORE_TIME;
          } else {
            roller_state = Off;
            state_duration = 1e10;
          }
          break;

        case NoDiscard:
          roller_state = Off;
          state_duration = 1e10;
          break;
      }
    }
    // We're unsure of which ball we have, so we'll score it by default
    else {
      roller_state = OutTop;
      state_duration = SCORE_TIME;
    }

    ball_in_lower_rollers = read_optical(&BallSensor);
    time_since_last_state = 0;
  } else {
    if (
      ball_in_lower_rollers == oc &&
      roller_state == OutTop &&
      time_since_last_state > MIN_STATE_GRACE_PERIOD
    ) {
      roller_state = TopOnly;
    }
  }

  rollers(roller_state);
}

void CycleState::reset() {
  ball_in_lower_rollers = None;
  roller_state = Off;
  state_duration = 0;
  time_since_last_state = 0;
  num_scored = 0;
}

CycleState cycle_state;

void drive(double left, double right, vex::brakeType brake_type) {
  spin(&BaseLeftRear, left, brake_type);
  spin(&BaseLeftFront, left, brake_type);
  spin(&BaseRightRear, right, brake_type);
  spin(&BaseRightFront, right, brake_type);
}

task wiggle() {
  return task([]() {
    while(true) {
      drive(50, 50);
      wait(0.7, sec);
      drive(0, 0, coast);
      wait(0.1, sec);
      drive(-50, -50);
      wait(0.2, sec);
      drive(0, 0, coast);
      wait(0.1, sec);
    }
    return 0;
  });
}

void intake(IntakeRollerAction action, double veloc) {
  if (action == In) {
    veloc = -veloc;
  } else if (action != Out) {
    veloc = 0;
  }

  spin(&LeftIntake, veloc, coast);
  spin(&RightIntake, veloc, coast);
}

void rollers(IntakeRollerAction action, double veloc) {
  double upper, lower;

  switch (action) {
    case OutTop:
      upper = veloc;
      lower = veloc;
      break;

    case OutFront:
      upper = -veloc;
      lower = -veloc;
      break;

    case OutBack:
      upper = -veloc;
      lower = veloc;
      break;

    case TopOnly:
      upper = veloc;
      lower = 0;
      break;

    case BottomOnly:
      spin(&UpperRoller, 0, brake);
      spin(&LowerRoller, veloc, coast);
      return;

    default:
      upper = 0;
      lower = 0;
      break;
  }

  spin(&UpperRoller, upper, coast);
  spin(&LowerRoller, lower, coast);
}

task release_flap() {
  UpperRoller.startRotateFor(0.25, turns, 100, vex::velocityUnits::pct);
  LowerRoller.startRotateFor(0.25, turns, 100, vex::velocityUnits::pct);

  return task([]() {
    wait(350, msec);
    spin(&UpperRoller, -100);
    spin(&LowerRoller, -100);
    wait(500, msec);
    UpperRoller.stop(coast);
    LowerRoller.stop(coast);
    return 0;
  });
}

void cycle_n(int num_balls) {
  const double TICK_TIME = 0.01; // seconds

  // Number of balls scored
  int num_scored = 0;
  // The time since we last scored a ball
  double time_since_last_score = MIN_STATE_GRACE_PERIOD + EPSILON;

  // Start the rollers
  rollers(OutTop);

  while (num_scored < num_balls) {
    // If a ball is in the middle of the rollers, then increase the number of balls scored
    if (CycleTrigger.pressing() && time_since_last_score > MIN_STATE_GRACE_PERIOD) {
      num_scored += 1;
      time_since_last_score = 0;
      intake(Off);
    }

    // If we haven't scored a ball in a while, run the intakes
    if (time_since_last_score > SCORE_TIME) {
      intake(In);
    } else {
      intake(Off);
    }

    // Wait until the next cycle
    wait(TICK_TIME, sec);
    time_since_last_score += TICK_TIME;
  }

  // Fininsh scoring the last ball
  intake(Off);
  waitUntil(!CycleTrigger.pressing());
  rollers(TopOnly);
  wait(SCORE_TIME, sec);
  rollers(Off);
}

void intake_mid() {
  rollers(OutTop, 75);
  intake(In);
  waitUntil(CycleTrigger.pressing());
  rollers(Off);
  intake(Off);
}

void outtake_one() {
  rollers(BottomOnly, -100);
  intake(Out);
  wait(0.4, sec);
  rollers(Off);
  intake(Off);
}

bool intake_low(double max_time) {
  intake(In, 100);
  double total_time = 0;
  UpperRoller.stop(brake);
  //spin(&LowerRoller, 25);
  while (read_optical(&BallSensor) == None) {
    wait(0.005, sec);
    total_time += 0.005;

    if (total_time > max_time && max_time != 0) {
      intake(Off);
      return false;
    }
  }
  intake(Off);
  return true;
}

void lower_rollers(double time) {
  rollers(OutFront);
  wait(time, sec);
  rollers(Off);
};