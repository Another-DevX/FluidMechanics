#pragma once
#include "utility"
#include <vector>

struct Vec2 {
  float x, y;
};

struct SolidFaces {
  bool horizontal;
  bool vertical;
};

struct Cell {
  float density = 0.f;
  float pressure = 0.f;
  bool isSolid = false;
};

class Mesh {
public:
  Mesh(unsigned nx, unsigned ny, unsigned cellSize);

  unsigned cellSize() const;
  unsigned nx() const;
  unsigned ny() const;

  // Acceso a celdas - devuelve celda por defecto si está fuera de rango
  Cell &at(unsigned i, unsigned j);
  const Cell &at(unsigned i, unsigned j) const;

  // Acceso a velocidades
  float vx(unsigned i, unsigned j) const;
  float &vx(unsigned i, unsigned j);
  float vy(unsigned i, unsigned j) const;
  float &vy(unsigned i, unsigned j);
  SolidFaces isSolidCellOrNeighbors(unsigned i, unsigned j);

private:
  unsigned nx_, ny_;
  unsigned cellSize_;
  std::vector<Cell> cells_;
  std::vector<float> vx_;
  std::vector<float> vy_;

  // Celda dummy para retornar cuando se accede fuera de rango
  static Cell dummyCell_;
  static float dummyVel_;
};
