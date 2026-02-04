#include <cmath>
#include <vector>

#include "controller.cpp"

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
      : time_constant{time_constant},
        thrust_coefficient{thrust_coefficient},
        rotor_constant{rotor_constant},
        omega_b{omega_b},
        desired_speed{0},
        speed{0},
        thrust{0} {}

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
  // State
  std::vector<double> position;
  std::vector<double> velocity;
  double attitude;
  double angular_velocity;
  double rotational_inertia;
  // Physical Parameters
  double mass;
  double drag_coefficient;
  double reference_area;
  double thrust_coefficient;
  double rotor_time_constant;
  double rotor_constant;
  double omega_b;
  double air_density;
  double width_px;
  double height_px;
  double arm_length;
  // Rotors
  Rotor left_rotor;
  Rotor right_rotor;

  std::vector<double> last_action;
  // PID controllers for maintaining state between frames
  PIDController x_pos_pid;
  PIDController x_speed_pid;
  PIDController attitude_pid;
  PIDController attitude_speed_pid;
  PIDController y_pos_pid;
  PIDController y_speed_pid;

 public:
  Drone(std::vector<double> position, std::vector<double> velocity,
        double attitude, double angular_velocity, double mass,
        double rotational_inertia, double drag_coefficient,
        double reference_area, double thrust_coefficient,
        double rotor_time_constant, double rotor_constant, double omega_b)
      : position{position},
        velocity{velocity},
        attitude{attitude},
        angular_velocity{angular_velocity},
        mass{mass},
        rotational_inertia{rotational_inertia},
        drag_coefficient{drag_coefficient},
        reference_area{reference_area},
        thrust_coefficient{thrust_coefficient},
        rotor_time_constant{rotor_time_constant},
        rotor_constant{rotor_constant},
        omega_b{omega_b},
        left_rotor{rotor_time_constant, thrust_coefficient, rotor_constant,
                   omega_b},
        right_rotor{rotor_time_constant, thrust_coefficient, rotor_constant,
                    omega_b},
        air_density{1.225},
        width_px{50},
        height_px{10},
        last_action{0, 0},
        arm_length{0.25},
        x_pos_pid{0.5, 0.02, 0.2},
        x_speed_pid{0.3, 0.01, 0.25},
        attitude_pid{0.56, 0.16, 0.04},
        attitude_speed_pid{3.0, 0.2, 0.4},
        y_pos_pid{10.0, 5.0, 0.0},
        y_speed_pid{10.0, 5.0, 0.0} {}

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

    // Decompose the combined thrust from each thruster into its x and y
    // components
    std::vector<double> thrust_vector{
        std::sin(this->attitude) * (thrust_1 + thrust_2),
        -std::cos(this->attitude) * (thrust_1 + thrust_2)};

    // Force vector for gravity
    std::vector<double> gravitational_acceleration{0, 9.81};

    // relative velocity between drone and air with wind
    std::vector<double> air_relative_velocity = {
        this->velocity[0] - wind_vector[0], this->velocity[1] - wind_vector[1]};

    // Calculate magnitude of relative velocity of air and drone
    double air_relative_velocity_magnitude{0};
    for (auto component : air_relative_velocity) {
      air_relative_velocity_magnitude += std::pow(component, 2);
    }
    air_relative_velocity_magnitude =
        std::sqrt(air_relative_velocity_magnitude);

    // Calculate drag from relative velocity with air and drag parameters
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

    // Calculate acceleration from thrust vector, gravitational vector and air
    // resistance vector
    std::vector<double> acceleration = {
        thrust_vector[0] / this->mass + gravitational_acceleration[0] +
            drag[0] / this->mass,
        thrust_vector[1] / this->mass + gravitational_acceleration[1] +
            drag[1] / this->mass};

    // Perform physics step for velocity and acceleration
    this->velocity[0] = this->velocity[0] + acceleration[0] * dt;
    this->velocity[1] = this->velocity[1] + acceleration[1] * dt;
    this->position[0] = this->position[0] + this->velocity[0] * dt;
    this->position[1] = this->position[1] + this->velocity[1] * dt;

    // Calculate torque caused by different in thrust of two Rotors
    double torque = (thrust_1 - thrust_2) * this->arm_length;
    // Calculate the angular acceleration from the torque
    double angular_acceleration = torque / this->rotational_inertia;
    // Calculate the angular velocity from a physics step of angular
    // acceleartion
    this->angular_velocity = this->angular_velocity + angular_acceleration * dt;
    // Perform physics step on drones attitude from angular velocity
    this->attitude = this->attitude + this->angular_velocity * dt;

    // Keep the angle defined within -pi to pi
    if (this->attitude > M_PI) {
      this->attitude = -M_PI + std::fmod(this->attitude, M_PI);
    } else if (this->attitude < -M_PI) {
      this->attitude = M_PI + std::fmod(this->attitude, M_PI);
    }
  }

  // Return the state vector of the drone
  std::vector<double> get_state() {
    return std::vector<double>{this->position[0], this->position[1],
                               this->velocity[0], this->velocity[1],
                               this->attitude,    this->angular_velocity};
  }

  // Reset and setup a new drone
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
    // Define minimum and maximum thrust values
    double thrust_minimum = 0.2;
    double thrust_maximum = 0.8;

    // Distance between where we are and where we want to be
    double x_error = this->position[0] - target_position[0];
    // Calculate optimal horizontal speed based on how far away we are
    // horizontally
    double optimal_x_speed = -this->x_pos_pid.compute(x_error, dt);
    // Difference between current horizontal speed and the optimal horizontal
    // speed
    double x_speed_error = this->velocity[0] - optimal_x_speed;
    // Calculate desired rotation based on How far from optimal horizontal speed
    // we are
    double desired_rot = -this->x_speed_pid.compute(x_speed_error, dt);

    // Define the minimum and maximum rotation we want to see
    double rot_min = -(M_PI / 4);
    double rot_max = M_PI / 4;

    // Pin desired rotation within the min max bounds
    desired_rot = std::min(std::max(desired_rot, rot_min), rot_max);

    // Calculate how far from the desired attitude we are
    double rot_error = this->attitude - desired_rot;
    // Calculate optimal angular speed based on how far from desired attitude we
    // are
    double optimal_angular_speed = -this->attitude_pid.compute(rot_error, dt);
    // Calculate difference between the optmial angular speed and the current
    // angular speed
    double angular_speed_error = optimal_angular_speed - this->angular_velocity;
    // calculate a desired angular speed based on how far we are from desired
    // angular speed
    double desired_angular_speed =
        this->attitude_speed_pid.compute(angular_speed_error, dt);

    // Define the left thrust to be equal to the desired angular speed and the
    // right thrust to be the negative of this, causing a rotation of the drone
    // in the correct direction in order to move in the correct horizontal
    // direction
    double left_thrust = desired_angular_speed;
    double right_thrust = -left_thrust;

    // Calculate difference between current height and desired height
    double y_error = this->position[1] - target_position[1];
    // calculate desired vertical velocity based on error in height
    double desired_yvel = this->y_pos_pid.compute(-y_error, dt);
    // calculate difference between current vertical velocity and desired
    double yvel_error = this->velocity[1] - desired_yvel;
    // calculate a desired thrust based on how far from desired y velocity we
    // are
    double base_thrust = this->y_speed_pid.compute(yvel_error, dt);

    // clamp base vertical thrust between minimum and maximum
    base_thrust =
        std::min(std::max(base_thrust, thrust_minimum), thrust_maximum);

    // add the horizontal thrust values to the base thrust
    double thruster_left = base_thrust + left_thrust;
    double thruster_right = base_thrust + right_thrust;
    std::vector<double> thrust_vector{thruster_left, thruster_right};

    return thrust_vector;
  }
};
