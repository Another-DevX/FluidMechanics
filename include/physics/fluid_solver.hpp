#pragma once
#include "include/physics/mesh.hpp"

struct FluidSolverConfig {
  float rho = 1.f;
  int poissonIterations = 20;
};

class FluidSolver {
public:
  explicit FluidSolver(FluidSolverConfig cfg = {}) : cfg_(cfg) {}

  void initialize(Mesh &mesh);
  void step(Mesh &mesh, float dt);

  static float divergence(const Mesh &mesh, int i, int j);

private:
  FluidSolverConfig cfg_;

  static void apply_boundary_conditions(Mesh &mesh);
};
