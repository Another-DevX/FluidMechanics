#pragma once
#include "include/physics/mesh.hpp"
#include <SFML/Graphics.hpp>
#include <optional>
#include <vector>

enum class VisualizationMode { Velocity, Smoke, Divergence };

enum class BrushType { Velocity, Smoke, Solid };

class MeshDebugDraw {
public:
  MeshDebugDraw(); // carga font 1 vez

  void drawCells(sf::RenderWindow &window, const Mesh &mesh);
  void drawVelocities(sf::RenderWindow &window, const Mesh &mesh);
  void drawInterpolatedVelocities(sf::RenderWindow &window, const Mesh &mesh,
                                  int resolution = 3);
  void drawDivergence(sf::RenderWindow &window, const Mesh &mesh);
  void drawNumbers(sf::RenderWindow &window, const Mesh &mesh);
  void drawBrushCursor(sf::RenderWindow &window);

  // Manejo de interacción con el mouse para arrastrar flechas
  void handleEvent(const sf::Event &event, sf::RenderWindow &window,
                   Mesh &mesh);
  void update(sf::RenderWindow &window, Mesh &mesh);

  // Cambiar modo de visualización
  void setVisualizationMode(VisualizationMode mode) { mode_ = mode; }
  VisualizationMode getVisualizationMode() const { return mode_; }
  void cycleVisualizationMode() {
    switch (mode_) {
      case VisualizationMode::Velocity: mode_ = VisualizationMode::Smoke; break;
      case VisualizationMode::Smoke: mode_ = VisualizationMode::Divergence; break;
      case VisualizationMode::Divergence: mode_ = VisualizationMode::Velocity; break;
    }
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

  // Toggle tipo de brush (velocidad vs humo vs sólido)
  void cycleBrushType() { 
    switch (brushType_) {
      case BrushType::Velocity: brushType_ = BrushType::Smoke; break;
      case BrushType::Smoke: brushType_ = BrushType::Solid; break;
      case BrushType::Solid: brushType_ = BrushType::Velocity; break;
    }
  }
  BrushType getBrushType() const { return brushType_; }

private:
  sf::Font font_;
  VisualizationMode mode_ = VisualizationMode::Velocity;
  bool showInterpolated_ = false;
  bool showVectors_ = false;
  int interpolatedResolution_ = 3;

  // Modo brush
  bool brushMode_ = false;
  float brushRadius_ = 80.f;
  BrushType brushType_ = BrushType::Velocity;
  
  // Estado de brush para humo
  bool isBrushingSmoke_ = false;
  
  // Estado de brush para sólidos
  bool isBrushingSolid_ = false;

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
