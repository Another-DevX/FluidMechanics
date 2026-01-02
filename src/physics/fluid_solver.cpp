#include "include/physics/fluid_solver.hpp"
#include "algorithm"
#include "cmath"

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

  // const float h = mesh.cellSize();
  // const float width = mesh.nx() * h;
  // const float height = mesh.ny() * h;
  //
  // // Centro del dominio
  // const float cx = 0.f;
  // const float cy = 0.f;
  //
  // // Radio del humo (en unidades mundo)
  // const float radius = 0.6f * std::min(width, height);
  //
  // for (unsigned j = 0; j < mesh.ny(); ++j) {
  //   for (unsigned i = 0; i < mesh.nx(); ++i) {
  //
  //     if (mesh.at(i, j).isSolid) {
  //       mesh.smokeAt(i, j).density = 0.f;
  //       continue;
  //     }
  //
  //     // Centro de la celda
  //     float x = (i + 0.5f) * h - width * 0.5f;
  //     float y = (j + 0.5f) * h - height * 0.5f;
  //
  //     float dx = x - cx;
  //     float dy = y - cy;
  //
  //     float dist = std::sqrt(dx * dx + dy * dy);
  //
  //     if (dist < radius) {
  //       mesh.smokeAt(i, j).density = 1.f;
  //     } else {
  //       mesh.smokeAt(i, j).density = 0.f;
  //     }
  //   }
  // }
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

void advectiveVelocity(Mesh &mesh, float dt) {

  const float h = mesh.cellSize();
  const float width = mesh.nx() * h;
  const float height = mesh.ny() * h;

  std::vector<float> vx_tmp((mesh.nx() + 1) * mesh.ny());
  std::vector<float> vy_tmp(mesh.nx() * (mesh.ny() + 1));

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i <= mesh.nx(); ++i) {

      float x = i * h - width / 2.f;
      float y = (j + 0.5f) * h - height / 2.f;

      Vec2 vel = mesh.getVelocityAt(x, y);

      float x_prev = x - dt * vel.x;
      float y_prev = y - dt * vel.y;

      vx_tmp[i + (mesh.nx() + 1) * j] =
          mesh.sampleBilinearX(mesh.nx() + 1, mesh.ny(), h, x_prev, y_prev);
    }
  }

  for (unsigned j = 0; j <= mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {

      float x = (i + 0.5f) * h - width / 2.f;
      float y = j * h - height / 2.f;

      Vec2 vel = mesh.getVelocityAt(x, y);

      float x_prev = x - dt * vel.x;
      float y_prev = y - dt * vel.y;

      vy_tmp[i + mesh.nx() * j] =
          mesh.sampleBilinearY(mesh.nx(), mesh.ny() + 1, h, x_prev, y_prev);
    }
  }
  mesh.swapVx(vx_tmp);
  mesh.swapVy(vy_tmp);

  vx_tmp.clear();
  vy_tmp.clear();
}

void advectSmoke(Mesh &mesh, float dt) {

  const float h = mesh.cellSize();
  const float width = mesh.nx() * h;
  const float height = mesh.ny() * h;

  const float originX = -width * 0.5f;
  const float originY = -height * 0.5f;

  for (unsigned j = 0; j < mesh.ny(); ++j) {
    for (unsigned i = 0; i < mesh.nx(); ++i) {

      if (mesh.at(i, j).isSolid || i == 0 || j == 0 || i == mesh.nx() - 1 ||
          j == mesh.ny() - 1) {
        mesh.smokeTmpAt(i, j).density = mesh.smokeAt(i, j).density;
        continue;
      }

      float x = originX + (i + 0.5f) * h;
      float y = originY + (j + 0.5f) * h;

      Vec2 vel = mesh.getVelocityAt(x, y);

      float v2 = vel.x * vel.x + vel.y * vel.y;
      if (v2 < 1e-8f) {
        mesh.smokeTmpAt(i, j).density = mesh.smokeAt(i, j).density;
        continue;
      }

      float x_prev = x - dt * vel.x;
      float y_prev = y - dt * vel.y;

      x_prev =
          std::clamp(x_prev, originX + 0.5f * h, originX + width - 0.5f * h);
      y_prev =
          std::clamp(y_prev, originY + 0.5f * h, originY + height - 0.5f * h);

      float tx = (x_prev - originX) / width;
      float ty = (y_prev - originY) / height;

      mesh.smokeTmpAt(i, j).density = mesh.sampleSmokeBilinear(tx, ty);
    }
  }

  mesh.swapSmoke();
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

  advectiveVelocity(mesh, dt);
  advectSmoke(mesh, dt);
}
