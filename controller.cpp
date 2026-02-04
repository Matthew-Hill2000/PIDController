
#include <algorithm>
class PIDController {
 private:
  double kp;
  double ki;
  double kd;
  double integral_error{0};
  double previous_error{0};
  double min_output{-1.0};
  double max_output{1.0};

 public:
  PIDController(double kp, double ki, double kd) : kp{kp}, ki{ki}, kd{kd} {}

  double compute(double error, double dt) {
    this->integral_error += error * dt;
    double derivative = (error - this->previous_error) / dt;
    double output = (this->kp * error) + (this->ki * this->integral_error) +
                    (this->kd * derivative);
    this->previous_error = error;

    output = std::min(std::max(output, min_output), max_output);

    return output;
  }
};
