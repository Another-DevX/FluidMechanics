#include "include/scene.hpp"
#include "include/physics/fluid_solver.hpp"
#include "include/physics/mesh.hpp"
#include "include/render/mesh_debug_draw.hpp"

class SimpleScene : public Scene {
public:
  SimpleScene() : mesh(120, 100, 0.5f) { solver.initialize(mesh); }

  void handle_event(const sf::Event &event, sf::RenderWindow &window) override {
    debug.handleEvent(event, window, mesh);
  }

  void update(float dt, sf::RenderWindow &window) override {
    debug.update(window, mesh);
    solver.step(mesh, dt * timeMultiplier_);
  }

  void render(sf::RenderWindow &window) override {
    debug.drawCells(window, mesh);

    // Mostrar flechas y números solo si showVectors está activo
    if (debug.showVectors()) {
      // Mostrar grilla interpolada o velocidades originales según toggle
      if (debug.showInterpolated()) {
        debug.drawInterpolatedVelocities(window, mesh, 5);
      } else {
        debug.drawVelocities(window, mesh);
      }

      debug.drawNumbers(window, mesh);
    }

    debug.drawBrushCursor(window);
  }

private:
  Mesh mesh;
  FluidSolver solver;
  MeshDebugDraw debug;
  float timeMultiplier_ = 10.f;
};

Scene *create_scene() {
  static SimpleScene scene;
  return &scene;
}
