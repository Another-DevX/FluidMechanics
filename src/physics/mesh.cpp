#include "include/physics/mesh.hpp"
#include <algorithm>
#include <cmath>

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

void Mesh::swapVx(std::vector<float> data) { vx_.swap(data); }

void Mesh::swapVy(std::vector<float> data) { vy_.swap(data); }

SolidFaces Mesh::isSolidCellOrNeighbors(unsigned i, unsigned j) {

  bool isHorizontalSolid = false;
  bool isVerticalSolid = false;

  if (at(i, j).isSolid || at(i + 1, j).isSolid || at(i - 1, j).isSolid)
    isHorizontalSolid = true;
  if (at(i, j).isSolid || at(i, j + 1).isSolid || at(i, j - 1).isSolid)
    isVerticalSolid = true;

  return SolidFaces{isHorizontalSolid, isVerticalSolid};
}

static float sampleBilinear(const std::vector<float> &data, int countX,
                            int countY, float cellSize, float x, float y) {
  // Convert world → grid space
  float originX = -(countX - 1) * cellSize * 0.5f;
  float originY = -(countY - 1) * cellSize * 0.5f;

  float px = (x - originX) / cellSize;
  float py = (y - originY) / cellSize;

  int left = int(std::floor(px));
  int bottom = int(std::floor(py));

  left = std::clamp(left, 0, countX - 2);
  bottom = std::clamp(bottom, 0, countY - 2);

  int right = left + 1;
  int top = bottom + 1;

  float xFrac = std::clamp(px - left, 0.f, 1.f);
  float yFrac = std::clamp(py - bottom, 0.f, 1.f);

  float vBL = data[left + countX * bottom];
  float vBR = data[right + countX * bottom];
  float vTL = data[left + countX * top];
  float vTR = data[right + countX * top];

  float vBottom = vBL + (vBR - vBL) * xFrac;
  float vTop = vTL + (vTR - vTL) * xFrac;
  return vBottom + (vTop - vBottom) * yFrac;
}
const float Mesh::sampleBilinearX(int countX, int countY, float cellSize,
                                  float x, float y) const {
  return sampleBilinear(vx_, countX, countY, cellSize, x, y);
}

const float Mesh::sampleBilinearY(int countX, int countY, float cellSize,
                                  float x, float y) const {
  return sampleBilinear(vy_, countX, countY, cellSize, x, y);
}

Vec2 Mesh::getVelocityAt(float x, float y) const {
  float cs = float(cellSize_);
  float velX = sampleBilinear(vx_, nx_ + 1, ny_, cs, x, y);
  float velY = sampleBilinear(vy_, nx_, ny_ + 1, cs, x, y);
  return {velX, velY};
}
