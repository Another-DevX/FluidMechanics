#include "include/render/mesh_debug_draw.hpp"
#include "include/physics/fluid_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

float MeshDebugDraw::clamp01(float x) { return std::clamp(x, 0.0f, 1.0f); }

std::uint8_t MeshDebugDraw::to_u8(float x01) {
  return static_cast<std::uint8_t>(255.f * clamp01(x01));
}

MeshDebugDraw::MeshDebugDraw() { font_.openFromFile("arial.ttf"); }

void MeshDebugDraw::drawCells(sf::RenderWindow &window, const Mesh &mesh) {
  // Si estamos en modo Divergencia, delegar a drawDivergence
  if (mode_ == VisualizationMode::Divergence) {
    drawDivergence(window, mesh);
    return;
  }

  auto size = window.getSize();
  float W = float(size.x);
  float H = float(size.y);

  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());
  float cellSize = float(mesh.cellSize());

  // Calcular el centro del mundo para convertir coordenadas
  float halfW = float(mesh.nx()) * cellSize * 0.5f;
  float halfH = float(mesh.ny()) * cellSize * 0.5f;

  sf::RectangleShape cell({dx, dy});

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {

      if (mesh.at(i, j).isSolid) {
        cell.setFillColor(sf::Color(40, 40, 40));
        cell.setPosition({i * dx, j * dy});
        window.draw(cell);
        continue;
      }

      std::uint8_t r, g, b;

      if (mode_ == VisualizationMode::Smoke) {
        // Modo humo: blanco y negro basado en densidad
        float density = mesh.smokeAt(i, j).density;
        float t = clamp01(density); // asumir densidad en [0, 1]
        std::uint8_t gray = to_u8(t);
        r = g = b = gray;
      } else {
        // Modo Velocity: Calcular posición del centro de la celda en
        // coordenadas mundo
        float worldX = -halfW + (float(i) + 0.5f) * cellSize;
        float worldY = -halfH + (float(j) + 0.5f) * cellSize;

        // Obtener velocidad interpolada en el centro de la celda
        Vec2 vel = mesh.getVelocityAt(worldX, worldY);
        float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);

        // Normalizar velocidad: 0 = azul, alto = rojo
        float t = std::tanh(speed * 0.5f); // t en [0, 1]

        // Gradiente azul -> cian -> verde -> amarillo -> rojo
        if (t < 0.25f) {
          float s = t / 0.25f;
          r = 0;
          g = to_u8(s);
          b = 255;
        } else if (t < 0.5f) {
          float s = (t - 0.25f) / 0.25f;
          r = 0;
          g = 255;
          b = to_u8(1.f - s);
        } else if (t < 0.75f) {
          float s = (t - 0.5f) / 0.25f;
          r = to_u8(s);
          g = 255;
          b = 0;
        } else {
          float s = (t - 0.75f) / 0.25f;
          r = 255;
          g = to_u8(1.f - s);
          b = 0;
        }
      }

      cell.setFillColor(sf::Color(r, g, b));
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

void MeshDebugDraw::drawInterpolatedVelocities(sf::RenderWindow &window,
                                               const Mesh &mesh,
                                               int resolution) {
  float cellSize = float(mesh.cellSize());

  float halfW = float(mesh.nx()) * cellSize * 0.5f;
  float halfH = float(mesh.ny()) * cellSize * 0.5f;

  int numX = int(mesh.nx()) * resolution;
  int numY = int(mesh.ny()) * resolution;

  auto size = window.getSize();
  float W = float(size.x);
  float H = float(size.y);

  float scale = velocityScale_ * 0.5f;

  for (int y = 0; y <= numY; ++y) {
    for (int x = 0; x <= numX; ++x) {
      float tx = float(x) / float(numX);
      float ty = float(y) / float(numY);

      // Posición en mundo (centrada en 0)
      float worldX = -halfW + tx * 2.f * halfW;
      float worldY = -halfH + ty * 2.f * halfH;

      Vec2 vel = mesh.getVelocityAt(worldX, worldY);

      // Posición en pantalla
      sf::Vector2f pos(tx * W, ty * H);

      float mag = std::sqrt(vel.x * vel.x + vel.y * vel.y);
      if (mag < 1e-3f) {
        sf::CircleShape pt(2.f);
        pt.setOrigin({2.f, 2.f});
        pt.setPosition(pos);
        pt.setFillColor(sf::Color(100, 200, 100, 150));
        window.draw(pt);
      } else {
        sf::Vector2f tip = pos + sf::Vector2f(std::tanh(vel.x) * scale,
                                              std::tanh(vel.y) * scale);
        sf::Vector2f d = tip - pos;
        float L = std::sqrt(d.x * d.x + d.y * d.y);
        if (L > 2.f) {
          sf::Vector2f u = d / L;    // vector unitario
          sf::Vector2f n{-u.y, u.x}; // perpendicular

          float thickness = 2.f;
          float headLen = std::min(6.f, L * 0.4f);
          float headW = 6.f;

          sf::Vector2f neck = tip - u * headLen;

          // Cuerpo de la flecha
          float bodyLen = L - headLen;
          if (bodyLen > 0.5f) {
            sf::RectangleShape body({bodyLen, thickness});
            body.setOrigin({0.f, thickness * 0.5f});
            body.setPosition(pos);
            body.setRotation(sf::radians(std::atan2(d.y, d.x)));
            body.setFillColor(sf::Color(100, 255, 100, 200));
            window.draw(body);
          }

          // Cabeza de la flecha (triángulo)
          sf::ConvexShape head;
          head.setPointCount(3);
          head.setPoint(0, tip);
          head.setPoint(1, neck + n * (headW * 0.5f));
          head.setPoint(2, neck - n * (headW * 0.5f));
          head.setFillColor(sf::Color(100, 255, 100, 200));
          window.draw(head);
        }
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
void MeshDebugDraw::drawNumbers(sf::RenderWindow &window, const Mesh &mesh) {
  auto size = window.getSize();
  float W = float(size.x), H = float(size.y);
  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {
      if (mesh.at(i, j).isSolid)
        continue;

      float value = 0.f;

      // if (mode_ == VisualizationMode::Divergence) {
      //   value = FluidSolver::divergence(mesh, i, j);
      // } else {
      //   value = mesh.at(i, j).pressure;
      // }

      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.2f", value);

      sf::Text text(font_, buf, 12);
      text.setFillColor(sf::Color::White);
      text.setOutlineColor(sf::Color::Black);
      text.setOutlineThickness(1.f);

      auto bounds = text.getLocalBounds();
      float x = i * dx + 0.5f * dx - 0.5f * bounds.size.x - bounds.position.x;
      float y = j * dy + 0.5f * dy - 0.5f * bounds.size.y - bounds.position.y;
      text.setPosition({x, y});

      window.draw(text);
    }
  }
}

void MeshDebugDraw::drawDivergence(sf::RenderWindow &window, const Mesh &mesh) {
  auto size = window.getSize();
  float W = float(size.x);
  float H = float(size.y);

  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());

  // Encontrar los valores máximo y mínimo de divergencia para normalización
  float minDiv = 0.f, maxDiv = 0.f;
  bool firstCell = true;

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {
      if (mesh.at(i, j).isSolid)
        continue;

      float div = FluidSolver::divergence(mesh, i, j);
      if (firstCell) {
        minDiv = maxDiv = div;
        firstCell = false;
      } else {
        minDiv = std::min(minDiv, div);
        maxDiv = std::max(maxDiv, div);
      }
    }
  }

  float divRange = maxDiv - minDiv;
  if (divRange < 1e-6f)
    divRange = 1.f;

  sf::RectangleShape cell({dx, dy});

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {
      if (mesh.at(i, j).isSolid) {
        cell.setFillColor(sf::Color(40, 40, 40));
        cell.setPosition({i * dx, j * dy});
        window.draw(cell);
        continue;
      }

      float div = FluidSolver::divergence(mesh, i, j);
      // Normalizar divergencia a [0, 1] donde 0.5 es el medio
      float normalized = (div - minDiv) / divRange;

      std::uint8_t r, g, b;

      // Colormap: azul (negativo) -> blanco (cero) -> rojo (positivo)
      if (normalized < 0.5f) {
        // Azul a blanco
        float s = normalized * 2.f; // [0, 1]
        r = to_u8(s);
        g = to_u8(s);
        b = 255;
      } else {
        // Blanco a rojo
        float s = (normalized - 0.5f) * 2.f; // [0, 1]
        r = 255;
        g = to_u8(1.f - s);
        b = to_u8(1.f - s);
      }

      cell.setFillColor(sf::Color(r, g, b));
      cell.setPosition({i * dx, j * dy});
      window.draw(cell);
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

  // Tecla V para cambiar modo de visualización
  if (const auto *key = event.getIf<sf::Event::KeyPressed>()) {
    if (key->code == sf::Keyboard::Key::V) {
      cycleVisualizationMode();
    }
    // Tecla I para toggle grilla interpolada
    if (key->code == sf::Keyboard::Key::I) {
      toggleInterpolatedGrid();
    }
    // Tecla H para ocultar/mostrar flechas y números
    if (key->code == sf::Keyboard::Key::H) {
      toggleShowVectors();
    }
    // Tecla B para toggle modo brush
    if (key->code == sf::Keyboard::Key::B) {
      toggleBrushMode();
    }
    // Tecla T para cambiar tipo de brush (velocidad/humo/sólido)
    if (key->code == sf::Keyboard::Key::T) {
      cycleBrushType();
    }
    // Teclas [ y ] para ajustar radio del brush
    if (key->code == sf::Keyboard::Key::LBracket) {
      decreaseBrushRadius();
    }
    if (key->code == sf::Keyboard::Key::RBracket) {
      increaseBrushRadius();
    }
    // Tecla R para reiniciar/limpiar la grid
    if (key->code == sf::Keyboard::Key::R) {
      mesh.clear();
    }
  }

  // Detectar click del mouse
  if (const auto *mousePressed = event.getIf<sf::Event::MouseButtonPressed>()) {
    if (mousePressed->button == sf::Mouse::Button::Left) {
      sf::Vector2f mousePos(float(mousePressed->position.x),
                            float(mousePressed->position.y));
      brushStart_ = mousePos;

      if (brushMode_) {
        if (brushType_ == BrushType::Smoke) {
          // Modo brush de humo: marcar que estamos pintando humo
          isBrushingSmoke_ = true;
        } else if (brushType_ == BrushType::Solid) {
          // Modo brush de sólidos: marcar que estamos pintando sólidos
          isBrushingSolid_ = true;
        } else {
          // Modo brush de velocidad: recolectar todos los nodos dentro del
          // radio
          brushNodes_.clear();

          // Buscar en vx (aristas verticales)
          for (unsigned j = 0; j < mesh.ny(); ++j) {
            for (unsigned i = 0; i <= mesh.nx(); ++i) {
              sf::Vector2f origin(i * dx, (j + 0.5f) * dy);
              float distToOrigin =
                  std::sqrt((mousePos.x - origin.x) * (mousePos.x - origin.x) +
                            (mousePos.y - origin.y) * (mousePos.y - origin.y));

              if (distToOrigin < brushRadius_) {
                brushNodes_.push_back(
                    BrushNode{true, i, j, origin, mesh.vx(i, j)});
              }
            }
          }

          // Buscar en vy (aristas horizontales)
          for (unsigned j = 0; j <= mesh.ny(); ++j) {
            for (unsigned i = 0; i < mesh.nx(); ++i) {
              sf::Vector2f origin((i + 0.5f) * dx, j * dy);
              float distToOrigin =
                  std::sqrt((mousePos.x - origin.x) * (mousePos.x - origin.x) +
                            (mousePos.y - origin.y) * (mousePos.y - origin.y));

              if (distToOrigin < brushRadius_) {
                brushNodes_.push_back(
                    BrushNode{false, i, j, origin, mesh.vy(i, j)});
              }
            }
          }
        } // cierre de else (BrushType::Velocity)
      } else {
        // Modo normal: buscar el nodo más cercano
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
  }

  // Soltar el mouse
  if (const auto *mouseReleased =
          event.getIf<sf::Event::MouseButtonReleased>()) {
    if (mouseReleased->button == sf::Mouse::Button::Left) {
      dragState_.reset();
      brushNodes_.clear();
      isBrushingSmoke_ = false;
      isBrushingSolid_ = false;
    }
  }
}

void MeshDebugDraw::update(sf::RenderWindow &window, Mesh &mesh) {
  sf::Vector2i mousePosInt = sf::Mouse::getPosition(window);
  sf::Vector2f mousePos(float(mousePosInt.x), float(mousePosInt.y));

  auto size = window.getSize();
  float W = float(size.x);
  float H = float(size.y);
  float dx = W / float(mesh.nx());
  float dy = H / float(mesh.ny());

  // Modo brush de humo: añadir densidad a las celdas bajo el cursor
  if (isBrushingSmoke_) {
    for (unsigned j = 0; j < mesh.ny(); ++j) {
      for (unsigned i = 0; i < mesh.nx(); ++i) {
        // Centro de la celda en píxeles
        float cx = (float(i) + 0.5f) * dx;
        float cy = (float(j) + 0.5f) * dy;

        float dist = std::sqrt((mousePos.x - cx) * (mousePos.x - cx) +
                               (mousePos.y - cy) * (mousePos.y - cy));

        if (dist < brushRadius_) {
          // Añadir densidad con falloff suave desde el centro
          float falloff = 1.f - (dist / brushRadius_);
          float addAmount =
              0.05f * falloff; // cantidad de humo a añadir por frame
          mesh.smokeAt(i, j).density =
              std::min(1.f, mesh.smokeAt(i, j).density + addAmount);
        }
      }
    }
    return;
  }

  // Modo brush de sólidos: marcar celdas como sólidas
  if (isBrushingSolid_) {
    for (unsigned j = 0; j < mesh.ny(); ++j) {
      for (unsigned i = 0; i < mesh.nx(); ++i) {
        // Centro de la celda en píxeles
        float cx = (float(i) + 0.5f) * dx;
        float cy = (float(j) + 0.5f) * dy;

        float dist = std::sqrt((mousePos.x - cx) * (mousePos.x - cx) +
                               (mousePos.y - cy) * (mousePos.y - cy));

        if (dist < brushRadius_) {
          mesh.at(i, j).isSolid = true;
        }
      }
    }
    return;
  }

  // Modo brush de velocidad: actualizar dinámicamente los nodos bajo el cursor
  if (!brushNodes_.empty()) {
    sf::Vector2f delta = mousePos - brushStart_;

    // Recalcular qué nodos están bajo el brush en la posición actual
    std::vector<BrushNode> currentNodes;

    // Buscar en vx
    for (unsigned j = 0; j < mesh.ny(); ++j) {
      for (unsigned i = 0; i <= mesh.nx(); ++i) {
        sf::Vector2f origin(i * dx, (j + 0.5f) * dy);
        float distToOrigin =
            std::sqrt((mousePos.x - origin.x) * (mousePos.x - origin.x) +
                      (mousePos.y - origin.y) * (mousePos.y - origin.y));

        if (distToOrigin < brushRadius_) {
          // Buscar si ya teníamos este nodo
          bool found = false;
          for (const auto &node : brushNodes_) {
            if (node.isVx && node.i == i && node.j == j) {
              currentNodes.push_back(node);
              found = true;
              break;
            }
          }
          if (!found) {
            // Nuevo nodo: guardarlo con su velocidad actual como original
            currentNodes.push_back(
                BrushNode{true, i, j, origin, mesh.vx(i, j)});
          }
        }
      }
    }

    // Buscar en vy
    for (unsigned j = 0; j <= mesh.ny(); ++j) {
      for (unsigned i = 0; i < mesh.nx(); ++i) {
        sf::Vector2f origin((i + 0.5f) * dx, j * dy);
        float distToOrigin =
            std::sqrt((mousePos.x - origin.x) * (mousePos.x - origin.x) +
                      (mousePos.y - origin.y) * (mousePos.y - origin.y));

        if (distToOrigin < brushRadius_) {
          bool found = false;
          for (const auto &node : brushNodes_) {
            if (!node.isVx && node.i == i && node.j == j) {
              currentNodes.push_back(node);
              found = true;
              break;
            }
          }
          if (!found) {
            currentNodes.push_back(
                BrushNode{false, i, j, origin, mesh.vy(i, j)});
          }
        }
      }
    }

    // Aplicar delta a todos los nodos actuales
    for (const auto &node : currentNodes) {
      float pixelDist;
      if (node.isVx) {
        pixelDist = delta.x;
      } else {
        pixelDist = delta.y;
      }

      // Multiplicador para velocidades más altas
      float velMultiplier = 5.f;
      float normalized =
          std::clamp(pixelDist / velocityScale_, -0.999f, 0.999f);
      float velDelta = std::atanh(normalized) * velMultiplier;
      float newVel = node.originalVel + velDelta;

      if (node.isVx) {
        mesh.vx(node.i, node.j) = newVel;
      } else {
        mesh.vy(node.i, node.j) = newVel;
      }
    }

    brushNodes_ = std::move(currentNodes);
    return;
  }

  // Modo normal: actualizar un solo nodo
  if (!dragState_)
    return;

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
  float velMultiplier = 5.f;
  float normalized = std::clamp(pixelDist / velocityScale_, -0.999f, 0.999f);
  float newVel = std::atanh(normalized) * velMultiplier;

  // Aplicar la nueva velocidad
  if (dragState_->isVx) {
    mesh.vx(dragState_->i, dragState_->j) = newVel;
  } else {
    mesh.vy(dragState_->i, dragState_->j) = newVel;
  }
}

void MeshDebugDraw::drawBrushCursor(sf::RenderWindow &window) {
  if (!brushMode_)
    return;

  sf::Vector2i mousePosInt = sf::Mouse::getPosition(window);
  sf::Vector2f mousePos(float(mousePosInt.x), float(mousePosInt.y));

  sf::CircleShape circle(brushRadius_);
  circle.setOrigin({brushRadius_, brushRadius_});
  circle.setPosition(mousePos);

  if (brushType_ == BrushType::Smoke) {
    // Color blanco para brush de humo
    circle.setFillColor(sf::Color(255, 255, 255, 30));
    circle.setOutlineColor(sf::Color(255, 255, 255, 180));
  } else if (brushType_ == BrushType::Solid) {
    // Color rojo para brush de sólidos
    circle.setFillColor(sf::Color(255, 50, 50, 30));
    circle.setOutlineColor(sf::Color(255, 50, 50, 180));
  } else {
    // Color amarillo para brush de velocidad
    circle.setFillColor(sf::Color(255, 255, 0, 30));
    circle.setOutlineColor(sf::Color(255, 255, 0, 180));
  }
  circle.setOutlineThickness(2.f);
  window.draw(circle);
}
