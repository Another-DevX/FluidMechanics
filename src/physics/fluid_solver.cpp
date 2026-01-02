#include "include/physics/fluid_solver.hpp"

float FluidSolver::divergence(const Mesh &mesh, int i, int j) {
  float dvdx = mesh.vx(i + 1, j) - mesh.vx(i, j);
  float dvdy = mesh.vy(i, j + 1) - mesh.vy(i, j);
  return dvdx + dvdy;
}

void FluidSolver::apply_boundary_conditions(Mesh &mesh) {}

void FluidSolver::initialize(Mesh &mesh) {

  for (unsigned i = 0; i < mesh.nx(); ++i) {
    mesh.at(i, 0).isSolid = true;
    mesh.at(i, mesh.ny() - 1).isSolid = true;
  }

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    mesh.at(0, j).isSolid = true;
    mesh.at(mesh.nx() - 1, j).isSolid = true;
  }

  mesh.vx(4, 3) = 1.f;
}

void solvePressureCell(Mesh &mesh, float dt, float rho, unsigned i,
                       unsigned j) {

  const float h = mesh.cellSize();

  int flowTop = mesh.at(i, j + 1).isSolid ? 0 : 1;
  int flowBottom = mesh.at(i, j - 1).isSolid ? 0 : 1;
  int flowLeft = mesh.at(i - 1, j).isSolid ? 0 : 1;
  int flowRight = mesh.at(i + 1, j).isSolid ? 0 : 1;

  int fluidEdgeCount = flowTop + flowRight + flowBottom + flowLeft;

  if (mesh.at(i, j).isSolid || fluidEdgeCount == 0) {
    mesh.at(i, j).pressure = 0.f;
    return;
  }

  float pressureTop = mesh.at(i, j + 1).pressure * flowTop;
  float pressureBottom = mesh.at(i, j - 1).pressure * flowBottom;
  float pressureLeft = mesh.at(i - 1, j).pressure * flowLeft;
  float pressureRight = mesh.at(i + 1, j).pressure * flowRight;

  float velocityTop = mesh.vy(i, j + 1) * flowTop;
  float velocityBottom = mesh.vy(i, j) * flowBottom;
  float velocityLeft = mesh.vx(i, j) * flowLeft;
  float velocityRight = mesh.vx(i + 1, j) * flowRight;

  float pressureSum =
      (pressureTop + pressureBottom) + (pressureRight + pressureLeft);
  float velocitySum =
      (velocityTop - velocityBottom) + (velocityRight - velocityLeft);

  mesh.at(i, j).pressure =
      (pressureSum - (h * rho * velocitySum) / dt) / fluidEdgeCount;
}

void adjustVelocity(Mesh &mesh, float dt, float rho, unsigned i, unsigned j) {

  auto [isHorizontalSolid, isVerticalSolid] = mesh.isSolidCellOrNeighbors(i, j);

  const float h = mesh.cellSize();
  float K = dt / (rho * h);

  if (isHorizontalSolid) {
    mesh.vx(i, j) = 0.f;
  } else {
    float gradpx = mesh.at(i, j).pressure - mesh.at(i - 1, j).pressure;
    mesh.vx(i, j) -= K * gradpx;
  }

  if (isVerticalSolid) {
    mesh.vy(i, j) = 0.f;
  } else {
    float gradpy = mesh.at(i, j).pressure - mesh.at(i, j - 1).pressure;
    mesh.vy(i, j) -= K * gradpy;
  }
}

void advectiveVelocity(Mesh &mesh, float dt, unsigned i, unsigned j) {

  const float h = mesh.cellSize();
}

void FluidSolver::step(Mesh &mesh, float dt) {
  const float rho = cfg_.rho;
  const unsigned nx = mesh.nx();
  const unsigned ny = mesh.ny();

  for (int it = 0; it < cfg_.poissonIterations; ++it) {
    for (unsigned j = 0; j < ny; ++j)
      for (unsigned i = 0; i < nx; ++i)
        solvePressureCell(mesh, dt, rho, i, j);
  }

  for (unsigned j = 0; j < ny; ++j) {
    for (unsigned i = 0; i < nx; ++i) {
      adjustVelocity(mesh, dt, rho, i, j);
    }
  }
}
