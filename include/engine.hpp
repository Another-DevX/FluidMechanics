#pragma once
#include "scene.hpp"
#include <SFML/Graphics.hpp>

class Engine {

public:
  Engine(uint32_t width, uint32_t height, const char *title);
  void run(Scene &scene);

private:
  sf::RenderWindow window;
};
