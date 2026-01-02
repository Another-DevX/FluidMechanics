#pragma once
#include <SFML/Graphics.hpp>

class Scene {
public:
  virtual ~Scene() = default;

  virtual void update(float dt, sf::RenderWindow &window) = 0;
  virtual void render(sf::RenderWindow &window) = 0;
  virtual void handle_event(const sf::Event &, sf::RenderWindow &window) {};
};
