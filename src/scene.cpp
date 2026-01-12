#include "include/scene.hpp"
#include "include/physics/fluid_solver.hpp"
#include "include/physics/mesh.hpp"
#include "include/render/mesh_debug_draw.hpp"

class SimpleScene : public Scene {
public:
  SimpleScene() : mesh(120, 100, 1.f) {
    solver.initialize(mesh);

    Jet inlet;
    inlet.enabled = true;
    inlet.i = 2;
    inlet.j = 45;
    inlet.width = 10;
    inlet.height = 5;
    inlet.vx = 6.f; // dirección del jet
    inlet.vy = 0.f;
    inlet.smokeDensity = 1.f;
    mesh.addJet(inlet);
  }

  void handle_event(const sf::Event &event, sf::RenderWindow &window) override {
    debug.handleEvent(event, window, mesh);
  }

  void update(float dt, sf::RenderWindow &window) override {
    debug.update(window, mesh);

    // 1) Inyectar velocidad del jet en el campo actual
    mesh.applyJetsVelocity(dt * timeMultiplier_);

    // 2) Paso de simulación (presión, proyección, advección de velocidad)
    solver.step(mesh, dt * timeMultiplier_);

    // 3) Inyectar humo del jet después de que el campo se estabilice
    mesh.applyJetsSmoke(dt * timeMultiplier_);
  }

  void render(sf::RenderWindow &window) override {
    debug.drawCells(window, mesh);

    // Mostrar flechas y números solo si showVectors está activo y no estamos en
    // modo divergencia
    if (debug.showVectors() &&
        debug.getVisualizationMode() != VisualizationMode::Divergence) {
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
  float timeMultiplier_ = 1.f;
};

Scene *create_scene() {
  static SimpleScene scene;
  return &scene;
}
