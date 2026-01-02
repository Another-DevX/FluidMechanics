#include "include/render/mesh_debug_draw.hpp"
#include "include/physics/fluid_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

float MeshDebugDraw::clamp01(float x) { return std::clamp(x, 0.0f, 1.0f); }

std::uint8_t MeshDebugDraw::to_u8(float x01) {
  return static_cast<std::uint8_t>(255.f * clamp01(x01));
}

MeshDebugDraw::MeshDebugDraw() { font_.openFromFile("arial.ttf"); }

void MeshDebugDraw::drawCells(sf::RenderWindow &window, const Mesh &mesh) {
  auto size = window.getSize();
  float W = float(size.x);
  float H = float(size.y);

  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());

  sf::RectangleShape cell({dx, dy});

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {

      if (mesh.at(i, j).isSolid) {
        cell.setFillColor(sf::Color(0, 20, 20));
        cell.setPosition({i * dx, j * dy});
        window.draw(cell);
        continue;
      }

      float div = FluidSolver::divergence(mesh, i, j);

      // s ∈ (-1, 1), centrado en 0 (negativo/positivo)
      float s = std::tanh(div); // si quieres más contraste: tanh(k*div)

      std::uint8_t r = to_u8(std::max(0.f, s));  // rojo si s>0
      std::uint8_t b = to_u8(std::max(0.f, -s)); // azul si s<0

      cell.setFillColor(sf::Color(r, 0, b));
      cell.setPosition({i * dx, j * dy});
      window.draw(cell);
    }
  }
}

void MeshDebugDraw::drawVelocities(sf::RenderWindow &window, const Mesh &mesh) {
  auto size = window.getSize();
  float W = float(size.x);
  float H = float(size.y);

  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());

  float scale = velocityScale_;
  auto vis = [](float v, float s) { return std::tanh(v) * s; };

  // vx (aristas verticales)
  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i <= mesh.nx(); ++i) {
      float v = mesh.vx(i, j);
      sf::Vector2f a(i * dx, (j + 0.5f) * dy);

      if (std::abs(v) < 1e-3f) {
        // Dibujar punto si la velocidad es ~0
        drawPoint(window, a);
      } else {
        sf::Vector2f b = a + sf::Vector2f(vis(v, scale), 0.f);
        drawArrow(window, a, b);
      }
    }
  }

  // vy (aristas horizontales)
  for (unsigned j = 0; j <= mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {
      float v = mesh.vy(i, j);
      sf::Vector2f a((i + 0.5f) * dx, j * dy);

      if (std::abs(v) < 1e-3f) {
        // Dibujar punto si la velocidad es ~0
        drawPoint(window, a);
      } else {
        sf::Vector2f b = a + sf::Vector2f(0.f, vis(v, scale));
        drawArrow(window, a, b);
      }
    }
  }
}
void MeshDebugDraw::drawArrow(sf::RenderWindow &window, sf::Vector2f a,
                              sf::Vector2f b, float thickness, float head_len,
                              float head_w) {
  sf::Vector2f d = b - a;
  float L = std::sqrt(d.x * d.x + d.y * d.y);
  if (L < 1e-4f)
    return;

  sf::Vector2f u = d / L;
  float angRad = std::atan2(u.y, u.x);

  // Ajusta cabeza si la flecha es muy corta
  float hl = std::min(head_len, 0.6f * L);
  sf::Vector2f neck = b - u * hl;

  // ---- cuerpo (barra gruesa) ----
  float bodyLen = std::sqrt((neck.x - a.x) * (neck.x - a.x) +
                            (neck.y - a.y) * (neck.y - a.y));

  sf::RectangleShape body({bodyLen, thickness});
  body.setOrigin({0.f, thickness * 0.5f});
  body.setPosition(a);

  // SFML 3: rotación usa sf::Angle
  body.setRotation(sf::radians(angRad));

  body.setFillColor(sf::Color::Yellow);
  window.draw(body);

  // ---- cabeza (triángulo relleno) ----
  sf::Vector2f n{-u.y, u.x}; // perpendicular

  sf::ConvexShape head;
  head.setPointCount(3);
  head.setPoint(0, b);
  head.setPoint(1, neck + n * (head_w * 0.5f));
  head.setPoint(2, neck - n * (head_w * 0.5f));
  head.setFillColor(sf::Color::Yellow);
  window.draw(head);
}
void MeshDebugDraw::drawDivergenceNumbers(sf::RenderWindow &window,
                                          const Mesh &mesh) {
  auto size = window.getSize();
  float W = float(size.x), H = float(size.y);
  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {
      float divv = FluidSolver::divergence(mesh, i, j);

      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.1f", divv);
      sf::Font font("arial.ttf");
      sf::Text text(font, buf, 14);

      auto bounds = text.getLocalBounds();
      float x = i * dx + 0.5f * dx - 0.5f * bounds.size.x - bounds.position.x;
      float y = j * dy + 0.5f * dy - 0.5f * bounds.size.y - bounds.position.y;
      text.setPosition({x, y});

      window.draw(text);
    }
  }
}

void MeshDebugDraw::drawPoint(sf::RenderWindow &window, sf::Vector2f pos,
                              float radius) {
  sf::CircleShape point(radius);
  point.setOrigin({radius, radius});
  point.setPosition(pos);
  point.setFillColor(sf::Color::Yellow);
  window.draw(point);
}

void MeshDebugDraw::handleEvent(const sf::Event &event,
                                sf::RenderWindow &window, Mesh &mesh) {
  auto size = window.getSize();
  float W = float(size.x);
  float H = float(size.y);
  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());

  // Detectar click del mouse
  if (const auto *mousePressed = event.getIf<sf::Event::MouseButtonPressed>()) {
    if (mousePressed->button == sf::Mouse::Button::Left) {
      sf::Vector2f mousePos(float(mousePressed->position.x),
                            float(mousePressed->position.y));

      float bestDist = 15.f; // radio de detección en píxeles
      std::optional<DragState> bestHit;

      // Buscar en vx (aristas verticales)
      for (unsigned j = 0; j < mesh.ny(); ++j) {
        for (unsigned i = 0; i <= mesh.nx(); ++i) {
          sf::Vector2f origin(i * dx, (j + 0.5f) * dy);
          float v = mesh.vx(i, j);
          sf::Vector2f tip =
              origin + sf::Vector2f(std::tanh(v) * velocityScale_, 0.f);

          // Verificar distancia al tip o al origin (para velocidad 0)
          float distToTip =
              std::sqrt((mousePos.x - tip.x) * (mousePos.x - tip.x) +
                        (mousePos.y - tip.y) * (mousePos.y - tip.y));
          float distToOrigin =
              std::sqrt((mousePos.x - origin.x) * (mousePos.x - origin.x) +
                        (mousePos.y - origin.y) * (mousePos.y - origin.y));
          float dist = std::min(distToTip, distToOrigin);

          if (dist < bestDist) {
            bestDist = dist;
            bestHit = DragState{true, i, j, origin};
          }
        }
      }

      // Buscar en vy (aristas horizontales)
      for (unsigned j = 0; j <= mesh.ny(); ++j) {
        for (unsigned i = 0; i < mesh.nx(); ++i) {
          sf::Vector2f origin((i + 0.5f) * dx, j * dy);
          float v = mesh.vy(i, j);
          sf::Vector2f tip =
              origin + sf::Vector2f(0.f, std::tanh(v) * velocityScale_);

          float distToTip =
              std::sqrt((mousePos.x - tip.x) * (mousePos.x - tip.x) +
                        (mousePos.y - tip.y) * (mousePos.y - tip.y));
          float distToOrigin =
              std::sqrt((mousePos.x - origin.x) * (mousePos.x - origin.x) +
                        (mousePos.y - origin.y) * (mousePos.y - origin.y));
          float dist = std::min(distToTip, distToOrigin);

          if (dist < bestDist) {
            bestDist = dist;
            bestHit = DragState{false, i, j, origin};
          }
        }
      }

      dragState_ = bestHit;
    }
  }

  // Soltar el mouse
  if (const auto *mouseReleased =
          event.getIf<sf::Event::MouseButtonReleased>()) {
    if (mouseReleased->button == sf::Mouse::Button::Left) {
      dragState_.reset();
    }
  }
}

void MeshDebugDraw::update(sf::RenderWindow &window, Mesh &mesh) {
  if (!dragState_)
    return;

  sf::Vector2i mousePosInt = sf::Mouse::getPosition(window);
  sf::Vector2f mousePos(float(mousePosInt.x), float(mousePosInt.y));

  sf::Vector2f delta = mousePos - dragState_->origin;

  // Calcular nueva velocidad basada en la posición del mouse
  // Usamos atanh para invertir el tanh usado en la visualización
  float pixelDist;
  if (dragState_->isVx) {
    pixelDist = delta.x; // Solo componente X para vx
  } else {
    pixelDist = delta.y; // Solo componente Y para vy
  }

  // Convertir distancia en píxeles a velocidad
  // tanh(v) * scale = pixelDist => v = atanh(pixelDist / scale)
  float normalized = std::clamp(pixelDist / velocityScale_, -0.999f, 0.999f);
  float newVel = std::atanh(normalized);

  // Aplicar la nueva velocidad
  if (dragState_->isVx) {
    mesh.vx(dragState_->i, dragState_->j) = newVel;
  } else {
    mesh.vy(dragState_->i, dragState_->j) = newVel;
  }
}
