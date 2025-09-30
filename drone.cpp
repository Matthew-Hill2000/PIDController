#include "controller.cpp"
#include <cmath>
#include <iostream>
#include <ostream>
#include <vector>

class Rotor {
private:
  double time_constant;
  double thrust_coefficient;
  double rotor_constant;
  double omega_b;
  double desired_speed;
  double speed;
  double thrust;

public:
  Rotor(double time_constant, double thrust_coefficient, double rotor_constant,
        double omega_b)
      : time_constant{time_constant}, thrust_coefficient{thrust_coefficient},
        rotor_constant{rotor_constant}, omega_b{omega_b}, desired_speed{0},
        speed{0}, thrust{0} {}

  void step(double dt) {
    this->speed +=
        ((this->desired_speed - this->speed) / this->time_constant) * dt;
    this->thrust = this->thrust_coefficient * (this->speed) * (this->speed);
  }

  void set_throttle(double throttle) {
    this->desired_speed = throttle * this->rotor_constant + this->omega_b;
  }

  double get_thrust() { return this->thrust; }

  void reset() {
    this->desired_speed = 0;
    this->speed = 0;
    this->thrust = 0;
  }
};

class Drone {
private:
  std::vector<double> position;
  std::vector<double> velocity;
  double attitude;
  double angular_velocity;
  double mass;
  double rotational_inertia;
  double drag_coefficient;
  double reference_area;
  double thrust_coefficient;
  double rotor_time_constant;
  double rotor_constant;
  double omega_b;
  Rotor left_rotor;
  Rotor right_rotor;
  double air_density;
  double width_px;
  double height_px;
  std::vector<double> last_action;
  double arm_length;

public:
  Drone(std::vector<double> position, std::vector<double> velocity,
        double attitude, double angular_velocity, double mass,
        double rotational_inertia, double drag_coefficient,
        double reference_area, double thrust_coefficient,
        double rotor_time_constant, double rotor_constant, double omega_b)
      : position{position}, velocity{velocity}, attitude{attitude},
        angular_velocity{angular_velocity}, mass{mass},
        rotational_inertia{rotational_inertia},
        drag_coefficient{drag_coefficient}, reference_area{reference_area},
        thrust_coefficient{thrust_coefficient},
        rotor_time_constant{rotor_time_constant},
        rotor_constant{rotor_constant}, omega_b{omega_b},
        left_rotor{rotor_time_constant, thrust_coefficient, rotor_constant,
                   omega_b},
        right_rotor{rotor_time_constant, thrust_coefficient, rotor_constant,
                    omega_b},
        air_density{1.225}, width_px{50}, height_px{10}, last_action{0, 0},
        arm_length{0.25} {}

  void step(std::vector<double> action, double dt,
            std::vector<double> wind_vector) {

    double u1 = std::max(0.0, std::min(action[0], 1.0));
    double u2 = std::max(0.0, std::min(action[1], 1.0));

    this->last_action = std::vector<double>{u1, u2};

    this->left_rotor.set_throttle(u1);     
    this->right_rotor.set_throttle(u2);

    this->left_rotor.step(dt);
    this->right_rotor.step(dt);

    double thrust_1 = this->left_rotor.get_thrust();
    double thrust_2 = this->right_rotor.get_thrust();

    std::vector<double> thrust_vector{
        std::sin(this->attitude) * (thrust_1 + thrust_2),
        -std::cos(this->attitude) * (thrust_1 + thrust_2)};

    std::vector<double> gravitational_acceleration{0, 9.81};

    std::vector<double> air_relative_velocity = {
        this->velocity[0] - wind_vector[0], this->velocity[1] - wind_vector[1]};

    double air_relative_velocity_magnitude{0};

    for (auto component : air_relative_velocity) {
      air_relative_velocity_magnitude += std::pow(component, 2);
    }

    air_relative_velocity_magnitude =
        std::sqrt(air_relative_velocity_magnitude);

    std::vector<double> drag;
    if (air_relative_velocity_magnitude != 0) {
      double drag_multiplier = 0.5 * this->drag_coefficient *
                               this->reference_area * this->air_density *
                               std::pow(air_relative_velocity_magnitude, 2);

      std::vector<double> normalized_velocity = {
          air_relative_velocity[0] / air_relative_velocity_magnitude,
          air_relative_velocity[1] / air_relative_velocity_magnitude};

      drag = std::vector<double>{-normalized_velocity[0] * drag_multiplier,
                                 -normalized_velocity[1] * drag_multiplier};
    } else {
      drag = std::vector<double>{0, 0};
    }

    std::vector<double> acceleration = {
        thrust_vector[0] / this->mass + gravitational_acceleration[0] +
            drag[0] / this->mass,
        thrust_vector[1] / this->mass + gravitational_acceleration[1] +
            drag[1] / this->mass};

    this->velocity[0] = this->velocity[0] + acceleration[0] * dt;
    this->velocity[1] = this->velocity[1] + acceleration[1] * dt;
    this->position[0] = this->position[0] + this->velocity[0] * dt;
    this->position[1] = this->position[1] + this->velocity[1] * dt;

    double torque = (thrust_1 - thrust_2) * this->arm_length;
    double angular_acceleration = torque / this->rotational_inertia;
    this->angular_velocity = this->angular_velocity + angular_acceleration * dt;
    this->attitude = this->attitude + this->angular_velocity * dt;

    if (this->attitude > M_PI) {
      this->attitude = -M_PI + std::fmod(this->attitude, M_PI);
    } else if (this->attitude < -M_PI) {
      this->attitude = M_PI + std::fmod(this->attitude, M_PI);
    }
  }
  std::vector<double> get_state() {
    return std::vector<double>{this->position[0], this->position[1],
                               this->velocity[0], this->velocity[1],
                               this->attitude,    this->angular_velocity};
  }

  void reset(std::vector<double> position, std::vector<double> velocity,
             double attitude, double angular_velocity, double mass,
             double rotational_inertia, double drag_coefficient,
             double reference_area, double thrust_coefficient,
             double rotor_time_constant, double rotor_constant,
             double omega_b) {
    this->position = position;
    this->velocity = velocity;
    this->attitude = attitude;
    this->angular_velocity = angular_velocity;
    this->mass = mass;
    this->rotational_inertia = rotational_inertia;
    this->drag_coefficient = drag_coefficient;
    this->reference_area = reference_area;

    this->left_rotor =
        Rotor(rotor_time_constant, thrust_coefficient, rotor_constant, omega_b);
    this->right_rotor =
        Rotor(rotor_time_constant, thrust_coefficient, rotor_constant, omega_b);
    this->last_action = std::vector<double>{0, 0};
  }

  std::vector<double> controller(std::vector<double> target_position,
                                 double dt) {
    double thrust_minimum = 0.2;
    double thrust_maximum = 0.8;

    PIDController x_pos_pid(1.2, 0.16, 0.01);
    PIDController x_speed_pid(0.2, 0.1, 0.0065);
    PIDController attitude_pid(0.56, 0.16, 0.04);
    PIDController attitude_speed_pid(8.0, 1.0, 0.2);
    PIDController y_pos_pid(10.0, 5.0, 0.0);
    PIDController y_speed_pid(10.0, 5.0, 0.0);

    double x_error = this->position[0] - target_position[0];
    double optimal_x_speed = -x_pos_pid.compute(x_error, dt);
    double x_speed_error = this->velocity[0] - optimal_x_speed;
    double desired_rot = -x_speed_pid.compute(x_speed_error, dt);

    double rot_min = -(M_PI / 4);
    double rot_max = M_PI / 4;

    desired_rot = std::min(std::max(desired_rot, rot_min), rot_max);

    double rot_error = this->attitude - desired_rot;
    double optimal_angular_speed = -attitude_pid.compute(rot_error, dt);
    double angular_speed_error = optimal_angular_speed - this->angular_velocity;
    double desired_angular_speed =
        attitude_speed_pid.compute(angular_speed_error, dt);

    double left_thrust = desired_angular_speed;
    double right_thrust = -left_thrust;

    double y_error = this->position[1] - target_position[1];
    double desired_yvel = y_pos_pid.compute(-y_error, dt);
    double yvel_error = this->velocity[1] - desired_yvel;
    double base_thrust = y_speed_pid.compute(yvel_error, dt);

    base_thrust =
        std::min(std::max(base_thrust, thrust_minimum), thrust_maximum);

    double thruster_left = base_thrust + left_thrust;
    double thruster_right = base_thrust + right_thrust;
    std::vector<double> thrust_vector{thruster_left, thruster_right};

    return thrust_vector;
  }
};
