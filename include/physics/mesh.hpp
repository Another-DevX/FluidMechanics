#pragma once
#include "utility"
#include <vector>

struct Vec2
{
  float x, y;
};

struct Jet
{
  bool enabled = true;
  int i = 1, j = 1; // esquina inferior-izquierda (en indices de celda)
  int width = 1, height = 1;
  float vx = 0.f, vy = 0.f;
  float smokeDensity = 0.2f; // cantidad por segundo (o por paso, tú decides)
};

struct SolidFaces
{
  bool horizontal;
  bool vertical;
};

struct Cell
{
  float density = 0.f;
  float pressure = 0.f;
  bool isSolid = false;
};

struct SmokeCell
{
  float density = 0.f;
};

class Mesh
{
public:
  Mesh(unsigned nx, unsigned ny, unsigned cellSize);

  unsigned cellSize() const;
  unsigned nx() const;
  unsigned ny() const;

  Cell &at(unsigned i, unsigned j);
  const Cell &at(unsigned i, unsigned j) const;

  SmokeCell &smokeAt(unsigned i, unsigned j);
  const SmokeCell &smokeAt(unsigned i, unsigned j) const;

  SmokeCell &smokeTmpAt(unsigned i, unsigned j);
  const SmokeCell &smokeTmpAt(unsigned i, unsigned j) const;

  float vx(unsigned i, unsigned j) const;
  float &vx(unsigned i, unsigned j);
  float vy(unsigned i, unsigned j) const;
  float &vy(unsigned i, unsigned j);

  void swapVx(std::vector<float> data);
  void swapVy(std::vector<float> data);
  void swapSmoke();

  void clear();

  void addJet(const Jet &jet) { jets_.push_back(jet); }
  void clearJets() { jets_.clear(); }
  void applyJetsVelocity(float dt);
  void applyJetsSmoke(float dt);

  SolidFaces isSolidCellOrNeighbors(unsigned i, unsigned j);

  Vec2 getVelocityAt(float x, float y) const;
  const float sampleBilinearX(int countX, int countY, float cellSize,
                              float worldX, float worldY) const;

  const float sampleBilinearY(int countX, int countY, float cellSize,
                              float worldX, float worldY) const;

  const float sampleSmokeBilinear(float worldX, float worldY) const;

private:
  unsigned nx_, ny_;
  float cellSize_;
  std::vector<Cell> cells_;
  std::vector<SmokeCell> smokeCells_;
  std::vector<SmokeCell> smokeCellsTmp_;
  std::vector<float> vx_;
  std::vector<float> vy_;
  std::vector<Jet> jets_;

  // Celda dummy para retornar cuando se accede fuera de rango
  static SmokeCell dummySmokeCell_;
  static Cell dummyCell_;
  static float dummyVel_;
};
