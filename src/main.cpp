#include "include/engine.hpp"
#include "include/scene.hpp"

Scene *create_scene();

int main() {

  const uint32_t WINDOW_WIDTH = 3480;
  const uint32_t WINDOW_HEIGHT = 2160;

  Engine engine(WINDOW_WIDTH, WINDOW_HEIGHT, "Fluid Simulation");
  engine.run(*create_scene());
  return 0;
}
