#include "include/scene.hpp"
#include "include/physics/fluid_solver.hpp"
#include "include/physics/mesh.hpp"
#include "include/render/mesh_debug_draw.hpp"

class SimpleScene : public Scene {
public:
  SimpleScene() : mesh(10, 8, 1) { solver.initialize(mesh); }

  void handle_event(const sf::Event &event, sf::RenderWindow &window) override {
    debug.handleEvent(event, window, mesh);
  }

  void update(float dt, sf::RenderWindow &window) override {
    debug.update(window, mesh);
    solver.step(mesh, dt);
  }

  void render(sf::RenderWindow &window) override {
    debug.drawCells(window, mesh);
    debug.drawVelocities(window, mesh);
    debug.drawDivergenceNumbers(window, mesh);
  }

private:
  Mesh mesh;
  FluidSolver solver;
  MeshDebugDraw debug;
};

Scene *create_scene() {
  static SimpleScene scene;
  return &scene;
}
