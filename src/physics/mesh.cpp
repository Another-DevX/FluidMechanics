#include "include/physics/mesh.hpp"
#include <algorithm>
#include <cmath>

Cell Mesh::dummyCell_ = {0.f, 0.f, true};
SmokeCell Mesh::dummySmokeCell_ = {0.f};
float Mesh::dummyVel_ = 0.f;

Mesh::Mesh(unsigned nx, unsigned ny, unsigned cellSize)
    : nx_(nx ? nx : 1), ny_(ny ? ny : 1), cells_(nx_ * ny_),
      vx_((nx_ + 1) * ny_, 0.f), vy_(nx_ * (ny_ + 1), 0.f),
      cellSize_(cellSize ? cellSize : 1), smokeCells_(nx_ * ny_),
      smokeCellsTmp_(nx_ * ny_) {}

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

SmokeCell &Mesh::smokeAt(unsigned i, unsigned j) {
  if (i < nx_ && j < ny_) {
    return smokeCells_[i + nx_ * j];
  }
  return dummySmokeCell_;
}

const SmokeCell &Mesh::smokeAt(unsigned i, unsigned j) const {
  if (i < nx_ && j < ny_) {
    return smokeCells_[i + nx_ * j];
  }

  return dummySmokeCell_;
}

SmokeCell &Mesh::smokeTmpAt(unsigned i, unsigned j) {
  if (i < nx_ && j < ny_) {
    return smokeCellsTmp_[i + nx_ * j];
  }
  return dummySmokeCell_;
}

const SmokeCell &Mesh::smokeTmpAt(unsigned i, unsigned j) const {
  if (i < nx_ && j < ny_) {
    return smokeCellsTmp_[i + nx_ * j];
  }

  return dummySmokeCell_;
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

void Mesh::swapSmoke() { smokeCells_.swap(smokeCellsTmp_); }

void Mesh::clear() {
  // Limpiar velocidades
  std::fill(vx_.begin(), vx_.end(), 0.f);
  std::fill(vy_.begin(), vy_.end(), 0.f);

  // Limpiar celdas (presión y densidad)
  for (auto &cell : cells_) {
    cell.pressure = 0.f;
    cell.density = 0.f;
    // No tocamos isSolid para mantener los bordes
  }

  // Limpiar humo
  for (auto &smoke : smokeCells_) {
    smoke.density = 0.f;
  }
  for (auto &smoke : smokeCellsTmp_) {
    smoke.density = 0.f;
  }
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

void Mesh::applyJetsVelocity(float dt) {
  for (const Jet &jet : jets_) {
    if (!jet.enabled)
      continue;

    int x0 = std::max(jet.i, 1);
    int y0 = std::max(jet.j, 1);
    int x1 = std::min(jet.i + jet.width - 1, int(nx()) - 2);
    int y1 = std::min(jet.j + jet.height - 1, int(ny()) - 2);

    // Aplicar velocidad en todo el área del jet
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        if (at(x, y).isSolid)
          continue;

        // Campo de velocidad en todo el rectángulo del jet
        vx(x, y) += jet.vx;
        vx(x + 1, y) += jet.vx;
        vy(x, y) += jet.vy;
        vy(x, y + 1) += jet.vy;
      }
    }
  }
}

void Mesh::applyJetsSmoke(float dt) {
  for (const Jet &jet : jets_) {
    if (!jet.enabled)
      continue;

    int x0 = std::max(jet.i, 1);
    int y0 = std::max(jet.j, 1);
    int x1 = std::min(jet.i + jet.width - 1, int(nx()) - 2);
    int y1 = std::min(jet.j + jet.height - 1, int(ny()) - 2);

    int bandHeight = y1 - y0 + 1;
    int smokeHeight = std::max(1, bandHeight / 1); // 1/5 del alto
    int smokeY0 = y0 + (bandHeight - smokeHeight) / 2;
    int smokeY1 = smokeY0 + smokeHeight - 1;

    // Inyectar humo solo en la banda central
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        if (at(x, y).isSolid)
          continue;

        if (y >= smokeY0 && y <= smokeY1) {
          auto &smoke = smokeAt(x, y);
          smoke.density += jet.smokeDensity * dt;
          if (smoke.density > 1.f)
            smoke.density = 1.f;
        }
      }
    }
  }
}

const float Mesh::sampleBilinearX(int countX, int countY, float cellSize,
                                  float x, float y) const {
  return sampleBilinear(vx_, countX, countY, cellSize, x, y);
}

const float Mesh::sampleBilinearY(int countX, int countY, float cellSize,
                                  float x, float y) const {
  return sampleBilinear(vy_, countX, countY, cellSize, x, y);
}

const float Mesh::sampleSmokeBilinear(float u, float v) const {
  const int width = (int)nx_;
  const int height = (int)ny_;

  u -= 0.5f / (float)width;
  v -= 0.5f / (float)height;

  u = std::clamp(u, 0.f, 1.f);
  v = std::clamp(v, 0.f, 1.f);

  float px = u * (float)width;
  float py = v * (float)height;

  int x = (int)std::floor(px);
  int y = (int)std::floor(py);

  float fx = std::clamp(px - x, 0.f, 1.f);
  float fy = std::clamp(py - y, 0.f, 1.f);

  auto sample = [&](int sx, int sy) -> float {
    sx = std::clamp(sx, 0, width - 1);
    sy = std::clamp(sy, 0, height - 1);
    return smokeAt((unsigned)sx, (unsigned)sy).density;
  };

  float bl = sample(x, y);
  float br = sample(x + 1, y);
  float tl = sample(x, y + 1);
  float tr = sample(x + 1, y + 1);

  float top = tl + (tr - tl) * fx;
  float bottom = bl + (br - bl) * fx;
  return bottom + (top - bottom) * fy;
}

Vec2 Mesh::getVelocityAt(float x, float y) const {
  float cs = float(cellSize_);
  float velX = sampleBilinear(vx_, nx_ + 1, ny_, cs, x, y);
  float velY = sampleBilinear(vy_, nx_, ny_ + 1, cs, x, y);
  return {velX, velY};
}
