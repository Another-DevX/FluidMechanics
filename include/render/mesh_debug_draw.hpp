#pragma once
#include "include/physics/mesh.hpp"
#include <SFML/Graphics.hpp>
#include <optional>
#include <vector>

enum class VisualizationMode { Divergence, Pressure };

class MeshDebugDraw {
public:
  MeshDebugDraw(); // carga font 1 vez

  void drawCells(sf::RenderWindow &window, const Mesh &mesh);
  void drawVelocities(sf::RenderWindow &window, const Mesh &mesh);
  void drawInterpolatedVelocities(sf::RenderWindow &window, const Mesh &mesh,
                                  int resolution = 3);
  void drawNumbers(sf::RenderWindow &window, const Mesh &mesh);
  void drawBrushCursor(sf::RenderWindow &window);

  // Manejo de interacción con el mouse para arrastrar flechas
  void handleEvent(const sf::Event &event, sf::RenderWindow &window,
                   Mesh &mesh);
  void update(sf::RenderWindow &window, Mesh &mesh);

  // Cambiar modo de visualización
  void setVisualizationMode(VisualizationMode mode) { mode_ = mode; }
  VisualizationMode getVisualizationMode() const { return mode_; }
  void toggleVisualizationMode() {
    mode_ = (mode_ == VisualizationMode::Divergence)
                ? VisualizationMode::Pressure
                : VisualizationMode::Divergence;
  }

  // Toggle para mostrar grilla interpolada
  void toggleInterpolatedGrid() { showInterpolated_ = !showInterpolated_; }
  bool showInterpolated() const { return showInterpolated_; }

  // Toggle para ocultar flechas y números
  void toggleShowVectors() { showVectors_ = !showVectors_; }
  bool showVectors() const { return showVectors_; }

  // Toggle modo brush
  void toggleBrushMode() { brushMode_ = !brushMode_; }
  bool brushMode() const { return brushMode_; }
  void setBrushRadius(float r) { brushRadius_ = r; }
  float getBrushRadius() const { return brushRadius_; }
  void increaseBrushRadius(float delta = 10.f) {
    brushRadius_ = std::max(10.f, brushRadius_ + delta);
  }
  void decreaseBrushRadius(float delta = 10.f) {
    brushRadius_ = std::max(10.f, brushRadius_ - delta);
  }

private:
  sf::Font font_;
  VisualizationMode mode_ = VisualizationMode::Divergence;
  bool showInterpolated_ = false;
  bool showVectors_ = false;
  int interpolatedResolution_ = 3;

  // Modo brush
  bool brushMode_ = false;
  float brushRadius_ = 80.f;

  // Estado de brush activo
  struct BrushNode {
    bool isVx;
    unsigned i, j;
    sf::Vector2f origin;
    float originalVel;
  };
  std::vector<BrushNode> brushNodes_;
  sf::Vector2f brushStart_;

  static float clamp01(float x);
  static std::uint8_t to_u8(float x01);

  void drawArrow(sf::RenderWindow &window, sf::Vector2f a, sf::Vector2f b,
                 float thickness = 3.f, float head_len = 10.f,
                 float head_w = 8.f);

  void drawPoint(sf::RenderWindow &window, sf::Vector2f pos,
                 float radius = 4.f);

  // Estado de arrastre
  struct DragState {
    bool isVx;           // true = vx, false = vy
    unsigned i, j;       // índices de la velocidad
    sf::Vector2f origin; // punto de origen de la flecha
  };
  std::optional<DragState> dragState_;
  float velocityScale_ =
      60.f; // escala visual para convertir velocidad a píxeles
};
