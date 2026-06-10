#pragma once

#include <algorithm>
#include <cmath>

#include "controller.h"

struct Vec2 {
  double x, y;

  Vec2 operator+(Vec2 v) const { return {x + v.x, y + v.y}; }

  Vec2 operator-(Vec2 v) const { return {x - v.x, y - v.y}; }

  Vec2 operator*(double s) const { return {x * s, y * s}; }

  Vec2 operator/(double s) const { return {x / s, y / s}; }
};

struct RotorThrottle {
  double left;
  double right;
};

struct DroneState {
  Vec2 position;
  Vec2 velocity;

  double attitude;
  double angular_velocity;
};

struct DroneConfig {
  double mass;
  double rotational_inertia;

  double drag_coefficient;
  double reference_area;

  double thrust_coefficient;

  double rotor_time_constant;
  double rotor_constant;
  double omega_b;

  double air_density;

  double arm_length;
};

struct PIDGains {
  double kp;
  double ki;
  double kd;
};

struct AutoTunedDroneGains {
  PIDGains attitude_rate;
  PIDGains attitude;

  PIDGains vel_y;
  PIDGains pos_y;

  PIDGains vel_x;
  PIDGains pos_x;

  AutoTunedDroneGains(const DroneConfig& config, double dt) {
    constexpr double damping = 0.8;

    double rotor_bw = 1.0 / config.rotor_time_constant;

    double rate_bw = 0.4 * rotor_bw;

    double torque_gain = config.arm_length / config.rotational_inertia;

    attitude_rate.kp = rate_bw / torque_gain;

    attitude_rate.kd = 2 * damping * rate_bw / torque_gain;

    attitude_rate.ki = 0.05 * attitude_rate.kp;

    double attitude_bw = rate_bw / 5.0;

    attitude.kp = attitude_bw;

    attitude.kd = 2 * damping * attitude_bw;

    attitude.ki = 0.02 * attitude.kp;

    double hover_omega = config.omega_b + 0.5 * config.rotor_constant;

    double hover_thrust = config.thrust_coefficient * hover_omega * hover_omega;

    double thrust_gain = 2 * hover_thrust / config.mass;

    double vel_bw = attitude_bw / 4.0;

    vel_y.kp = vel_bw / thrust_gain;

    vel_y.kd = 2 * damping * vel_bw / thrust_gain;

    vel_y.ki = 0.4 * vel_y.kp;

    double pos_bw = vel_bw / 3.0;

    pos_y.kp = pos_bw;

    pos_y.kd = 0;

    pos_y.ki = 0.2 * pos_y.kp;

    double horiz_gain = hover_thrust / config.mass;

    vel_x.kp = vel_bw / horiz_gain;

    vel_x.kd = 2 * damping * vel_bw / horiz_gain;

    vel_x.ki = 0.02 * vel_x.kp;

    pos_x.kp = pos_bw;

    pos_x.kd = 0;

    pos_x.ki = 0.15 * pos_x.kp;
  }
};

class Rotor {
 private:
  double tau;
  double thrust_coeff;

  double rotor_constant;
  double omega_b;

  double desired_speed{0};
  double speed{0};

  double thrust{0};

 public:
  Rotor(double tau, double thrust_coeff, double rotor_constant, double omega_b)
      : tau(tau),
        thrust_coeff(thrust_coeff),
        rotor_constant(rotor_constant),
        omega_b(omega_b) {}

  void set_throttle(double u) { desired_speed = omega_b + rotor_constant * u; }

  void step(double dt) {
    speed += (desired_speed - speed) / tau * dt;

    thrust = thrust_coeff * speed * speed;
  }

  double get_thrust() const { return thrust; }
};

class Drone {
 private:
  DroneState state;
  DroneConfig config;

  AutoTunedDroneGains gains;

  Rotor left;
  Rotor right;

  PIDController rate_pid;
  PIDController attitude_pid;

  PIDController vel_y_pid;
  PIDController pos_y_pid;

  PIDController vel_x_pid;
  PIDController pos_x_pid;

 public:
  Drone(DroneState s, DroneConfig c, double dt)
      : state(s),
        config(c),

        gains(c, dt),

        left(c.rotor_time_constant, c.thrust_coefficient, c.rotor_constant,
             c.omega_b),

        right(c.rotor_time_constant, c.thrust_coefficient, c.rotor_constant,
              c.omega_b),

        rate_pid(gains.attitude_rate.kp, gains.attitude_rate.ki,
                 gains.attitude_rate.kd),

        attitude_pid(gains.attitude.kp, gains.attitude.ki, gains.attitude.kd),

        vel_y_pid(gains.vel_y.kp, gains.vel_y.ki, gains.vel_y.kd),

        pos_y_pid(gains.pos_y.kp, gains.pos_y.ki, gains.pos_y.kd),

        vel_x_pid(gains.vel_x.kp, gains.vel_x.ki, gains.vel_x.kd),

        pos_x_pid(gains.pos_x.kp, gains.pos_x.ki, gains.pos_x.kd) {}

  RotorThrottle controller(Vec2 target, double dt) {
    double x_err = target.x - state.position.x;

    double x_vel_cmd = pos_x_pid.compute(x_err, dt);

    double x_vel_err = x_vel_cmd - state.velocity.x;

    double rot_cmd = vel_x_pid.compute(x_vel_err, dt);

    double att_err = rot_cmd - state.attitude;

    double ang_vel_cmd = attitude_pid.compute(att_err, dt);

    double ang_vel_err = ang_vel_cmd - state.angular_velocity;

    double diff = rate_pid.compute(ang_vel_err, dt);

    double y_err = target.y - state.position.y;

    double y_vel_cmd = pos_y_pid.compute(y_err, dt);

    double y_vel_err = y_vel_cmd - state.velocity.y;

    double hover = 0.5;

    double thrust = hover + vel_y_pid.compute(y_vel_err, dt);

    thrust = std::clamp(thrust, 0.2, 0.8);

    return {

        thrust + diff,

        thrust - diff

    };
  }

  DroneState get_state() const { return state; }

  void step(RotorThrottle u, double dt, Vec2 wind) {
    left.set_throttle(u.left);
    right.set_throttle(u.right);

    left.step(dt);
    right.step(dt);

    double t1 = left.get_thrust();

    double t2 = right.get_thrust();

    Vec2 thrust_vec{

        sin(state.attitude) * (t1 + t2),

        -cos(state.attitude) * (t1 + t2)

    };

    Vec2 gravity{

        0, 9.81

    };

    Vec2 accel = thrust_vec / config.mass + gravity;

    state.velocity = state.velocity + accel * dt;

    state.position = state.position + state.velocity * dt;

    double torque = (t1 - t2) * config.arm_length;

    state.angular_velocity += torque / config.rotational_inertia * dt;

    state.attitude += state.angular_velocity * dt;
  }
};
