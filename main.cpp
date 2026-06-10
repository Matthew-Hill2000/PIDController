#include <SFML/Graphics.hpp>
#include <ctime>
#include <random>

#include "drone.h"

constexpr unsigned WINDOW_SIZE = 800;
constexpr float SCALE = 100.0f;

constexpr int FPS = 60;
constexpr double DT = 1.0 / FPS;

constexpr float DRONE_WIDTH = 50.0f;
constexpr float DRONE_HEIGHT = 10.0f;

constexpr float TARGET_RADIUS = 5.0f;

int main() {
  DroneConfig config;

  config.mass = 1.2;
  config.rotational_inertia = 0.6;

  config.drag_coefficient = 0.5;
  config.reference_area = 0.08;

  config.thrust_coefficient = 2.0e-07;

  config.rotor_time_constant = 0.05;
  config.rotor_constant = 6400.0;

  config.omega_b = 1800.0;

  config.air_density = 1.225;

  config.arm_length = 0.25;

  DroneState initial_state{{4, 4}, {0, 0}, 0.0, 0.0};

  Drone drone{initial_state, config, DT};

  Vec2 target{6.0, 3.0};

  Vec2 wind{0.0, 0.0};

  sf::RenderWindow window(sf::VideoMode({WINDOW_SIZE, WINDOW_SIZE}),
                          "Auto-Tuned Drone Simulation");

  window.setFramerateLimit(FPS);

  sf::RectangleShape drone_rect(sf::Vector2f(DRONE_WIDTH, DRONE_HEIGHT));

  drone_rect.setOrigin(sf::Vector2f{DRONE_WIDTH / 2, DRONE_HEIGHT / 2});
  drone_rect.setFillColor(sf::Color::White);

  sf::CircleShape target_circle(TARGET_RADIUS);

  target_circle.setFillColor(sf::Color::Green);

  auto set_target = [&](Vec2 pos) {
    target = pos;

    target_circle.setPosition(
        sf::Vector2f{static_cast<float>(pos.x * SCALE - TARGET_RADIUS),
                     static_cast<float>(pos.y * SCALE - TARGET_RADIUS)});
  };

  set_target(target);

  std::mt19937 rng(static_cast<unsigned>(std::time(nullptr)));

  std::uniform_real_distribution<double> x_dist(1.0, 7.0);

  std::uniform_real_distribution<double> y_dist(1.0, 7.0);

  while (window.isOpen()) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) window.close();

      if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
        if (key->code == sf::Keyboard::Key::Space) {
          set_target({x_dist(rng), y_dist(rng)});
        }
      }
    }

    RotorThrottle action = drone.controller(target, DT);

    drone.step(action, DT, wind);

    DroneState state = drone.get_state();

    drone_rect.setPosition(
        sf::Vector2f{static_cast<float>(state.position.x * SCALE),
                     static_cast<float>(state.position.y * SCALE)});
    drone_rect.setRotation(
        sf::degrees(static_cast<float>(state.attitude * 180.0 / M_PI)));
    window.clear();

    window.draw(target_circle);

    window.draw(drone_rect);

    window.display();
  }
}
