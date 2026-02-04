#include <SFML/Graphics.hpp>
#include <ctime>
#include <random>

#include "drone.cpp"

int main() {
  // Random number generator for target positions
  std::mt19937 rng(static_cast<unsigned>(std::time(nullptr)));
  std::uniform_real_distribution<double> x_dist(1.0, 7.0);
  std::uniform_real_distribution<double> y_dist(1.0, 7.0);

  double dt = 1.0 / 60.0;  // Time Step
  std::vector<double> target_position{6.0, 3.0};
  std::vector<double> wind_vector{0.0, 0.0};
  // create a drone object
  Drone drone{std::vector<double>{4, 4},
              std::vector<double>{0, 0},
              0.0,
              0.0,
              1.2,
              0.6,
              0.5,
              0.08,
              2.0e-07,
              0.05,
              6400.0,
              1800.0};

  // Setup a floating window
  sf::RenderWindow window(sf::VideoMode({800, 800}), "Drone Simulation",
                          sf::Style::Titlebar | sf::Style::Close);

  // set FPS
  window.setFramerateLimit(60);

  // scale floating window
  sf::Vector2u windowSize = window.getSize();
  float scale = 100.0f;

  // Create the drone rectangle representation
  sf::RectangleShape drone_rect(sf::Vector2f(50.f, 10.f));
  drone_rect.setFillColor(sf::Color::White);
  // Set origin to center for rotation
  drone_rect.setOrigin(sf::Vector2f(25.f, 5.f));

  // Create the target circle representation and set its position
  sf::CircleShape target(5.0f);
  target.setFillColor(sf::Color::Green);
  target.setPosition(sf::Vector2f(target_position[0] * scale - 5,
                                  target_position[1] * scale - 5));

  // keep checking for keyboard input
  while (window.isOpen()) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) window.close();

      // Move target position to new random location if space bar pressed
      if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
        if (keyPressed->code == sf::Keyboard::Key::Space) {
          // Generate new random target position
          target_position[0] = x_dist(rng);
          target_position[1] = y_dist(rng);
          target.setPosition(sf::Vector2f(target_position[0] * scale - 5,
                                          target_position[1] * scale - 5));
        }
      }
    }

    std::vector<double> action = drone.controller(target_position, dt);
    drone.step(action, dt, wind_vector);

    std::vector<double> state = drone.get_state();
    drone_rect.setPosition(sf::Vector2f(state[0] * scale, state[1] * scale));
    drone_rect.setRotation(
        sf::degrees(state[4] * 180.0f / M_PI));  // Convert radians to degrees

    // Render the drone
    window.clear();
    window.draw(target);
    window.draw(drone_rect);
    window.display();
  }
}
