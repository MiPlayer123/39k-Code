#ifndef __CONTROL_H
#define __CONTROL_H

#include "consts.h"
#include "vex.h"
#include "util.h"

using namespace vex;

#define LAST_ERROR_NICHE 1000

class DualPidController {
private:
  motor *m1;
  motor *m2;

  // Used for a specific task
  double target;
  double initial_error;
  double last_error;
  double integral;
  double last_output;
  double adjustment;
  int brake_ticks;

  // Constraints
  double min_veloc;
  double max_veloc;
  double max_accel;

  const double kp; // 1/s
  const double ki; // 1/s/s
  const double kd; // unitless
  const double dt; // s
  const double integral_threshold;

public:
  DualPidController(motor *m1, motor *m2, double kp, double ki, double kd, double dt, double integral_threshold);

  void set_rel_target(double new_target);

  void set_constraints(double minv, double maxv, double maxa);

  double error();

  bool is_done();

  void set_adjustment(double adj_value);

  void update(bool debug=false);
};

#endif