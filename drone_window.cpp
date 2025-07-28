#include "drone.cpp"
#include <SFML/Graphics.hpp>
#include <iostream>

int main() {

  double dt = 1.0 / 600.0;
  std::vector<double> target_position{9, 9.4};
  std::vector<double> wind_vector{0.0, 0.0};
  Drone drone{std::vector<double>{4, 4},
              std::vector<double>{0, 0},
              0,
              0,
              1.1346720670093304,
              0.45548851057993167,
              0.49251731396547266,
              0.0761621482944658,
              1.984e-07,
              0.050022585744253556,
              6432,
              1779};

  Drone test_drone{std::vector<double>{1, 3},
                   std::vector<double>{-0.2, 0},
                   -0.2,
                   0,
                   1.1346720670093304,
                   0.45548851057993167,
                   0.49251731396547266,
                   0.0761621482944658,
                   1.984e-07,
                   0.050022585744253556,
                   6432,
                   1779};

  std::vector<double> test_action =
      test_drone.controller(std::vector<double>{1, 3}, 0.1);
  sf::RenderWindow window(sf::VideoMode({800, 600}), "Drone Simulation");

  sf::Vector2u windowSize = window.getSize();
  float scale = 50.0f;

  sf::RectangleShape square(sf::Vector2f(20.f, 20.f));
  square.setFillColor(sf::Color::Red);

  sf::CircleShape target(5.0f);
  target.setFillColor(sf::Color::Green);
  target.setPosition(sf::Vector2f(target_position[0] * scale - 5,
                                  target_position[1] * scale - 5));

  while (window.isOpen()) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>())
        window.close();
    }

    std::vector<double> action = drone.controller(target_position, dt);
    drone.step(action, dt, wind_vector);

    std::vector<double> state = drone.get_state();
    square.setPosition(
        sf::Vector2f(state[0] * scale - 10, state[1] * scale - 10));

    window.clear();
    window.draw(target);
    window.draw(square);
    window.display();
  }
}
