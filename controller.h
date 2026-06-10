#pragma once
#include <algorithm>

class PIDController {

 private:

  double kp;
  double ki;
  double kd;

  double integral{0};
  double prev_error{0};

  double min_output{-1.0};
  double max_output{1.0};

 public:

  PIDController(double kp, double ki, double kd)
      : kp(kp), ki(ki), kd(kd) {}

  double compute(double error, double dt)
  {

    integral += error * dt;

    if (ki > 0)
      integral =
          std::clamp(
              integral,
              min_output / ki,
              max_output / ki);

    double derivative =
        (error - prev_error) / dt;

    prev_error = error;

    double output =
        kp * error
        + ki * integral
        + kd * derivative;

    return std::clamp(
        output,
        min_output,
        max_output);
  }

  void reset()
  {
    integral = 0;
    prev_error = 0;
  }
};
