#include "include/physics/mesh.hpp"

Cell Mesh::dummyCell_ = {0.f, 0.f, true};
float Mesh::dummyVel_ = 0.f;

Mesh::Mesh(unsigned nx, unsigned ny, unsigned cellSize)
    : nx_(nx ? nx : 1), ny_(ny ? ny : 1), cells_(nx_ * ny_),
      vx_((nx_ + 1) * ny_, 0.f), vy_(nx_ * (ny_ + 1), 0.f),
      cellSize_(cellSize ? cellSize : 1) {}

unsigned Mesh::cellSize() const { return cellSize_; }

unsigned Mesh::nx() const { return nx_; }

unsigned Mesh::ny() const { return ny_; }

Cell &Mesh::at(unsigned i, unsigned j) {
  if (i < nx_ && j < ny_) {
    return cells_[i + nx_ * j];
  }
  // Resetear dummy antes de retornar para evitar efectos secundarios
  dummyCell_ = {0.f, 0.f, true};
  return dummyCell_;
}

const Cell &Mesh::at(unsigned i, unsigned j) const {
  if (i < nx_ && j < ny_) {
    return cells_[i + nx_ * j];
  }
  // Para const, retornamos la dummy (que será 0,0)
  return dummyCell_;
}

float Mesh::vx(unsigned i, unsigned j) const {
  if (i <= nx_ && j < ny_) {
    return vx_[i + (nx_ + 1) * j];
  }
  return 0.f;
}

float &Mesh::vx(unsigned i, unsigned j) {
  if (i <= nx_ && j < ny_) {
    return vx_[i + (nx_ + 1) * j];
  }
  dummyVel_ = 0.f;
  return dummyVel_;
}

float Mesh::vy(unsigned i, unsigned j) const {
  if (i < nx_ && j <= ny_) {
    return vy_[i + nx_ * j];
  }
  return 0.f;
}

float &Mesh::vy(unsigned i, unsigned j) {
  if (i < nx_ && j <= ny_) {
    return vy_[i + nx_ * j];
  }
  dummyVel_ = 0.f;
  return dummyVel_;
}

SolidFaces Mesh::isSolidCellOrNeighbors(unsigned i, unsigned j) {

  bool isHorizontalSolid = false;
  bool isVerticalSolid = false;

  if (at(i, j).isSolid || at(i + 1, j).isSolid || at(i - 1, j).isSolid)
    isHorizontalSolid = true;
  if (at(i, j).isSolid || at(i, j + 1).isSolid || at(i, j - 1).isSolid)
    isVerticalSolid = true;

  return SolidFaces{isHorizontalSolid, isVerticalSolid};
}
