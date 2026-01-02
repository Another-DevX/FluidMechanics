#include "include/engine.hpp"
#include "include/scene.hpp"

Scene *create_scene();

int main() {

  const uint32_t WINDOW_WIDTH = 1080;
  const uint32_t WINDOW_HEIGHT = 720;

  Engine engine(WINDOW_WIDTH, WINDOW_HEIGHT, "Fluid Simulation");
  engine.run(*create_scene());
  return 0;
}
