#pragma once
#include <SFML/Graphics.hpp>
#include "include/physics/mesh.hpp"
#include <optional>

class MeshDebugDraw {
public:
  MeshDebugDraw(); // carga font 1 vez

  void drawCells(sf::RenderWindow& window, const Mesh& mesh);
  void drawVelocities(sf::RenderWindow& window, const Mesh& mesh);
  void drawDivergenceNumbers(sf::RenderWindow& window, const Mesh& mesh);

  // Manejo de interacción con el mouse para arrastrar flechas
  void handleEvent(const sf::Event& event, sf::RenderWindow& window, Mesh& mesh);
  void update(sf::RenderWindow& window, Mesh& mesh);

private:
  sf::Font font_;

  static float clamp01(float x);
  static std::uint8_t to_u8(float x01);

  void drawArrow(sf::RenderWindow& window,
                 sf::Vector2f a, sf::Vector2f b,
                 float thickness = 3.f,
                 float head_len = 10.f,
                 float head_w = 8.f);

  void drawPoint(sf::RenderWindow& window, sf::Vector2f pos, float radius = 4.f);

  // Estado de arrastre
  struct DragState {
    bool isVx;           // true = vx, false = vy
    unsigned i, j;       // índices de la velocidad
    sf::Vector2f origin; // punto de origen de la flecha
  };
  std::optional<DragState> dragState_;
  float velocityScale_ = 20.f; // escala visual para convertir velocidad a píxeles
};
