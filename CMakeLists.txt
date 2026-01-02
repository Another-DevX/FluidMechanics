cmake_minimum_required(VERSION 3.20)
project(fluid_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System Audio)

add_executable(sim
  src/main.cpp
  src/engine.cpp
  src/scene.cpp
  src/physics/fluid_solver.cpp
  src/physics/mesh.cpp
  src/render/mesh_debug_draw.cpp
)

target_include_directories(sim PRIVATE .)
target_link_libraries(sim PRIVATE
  SFML::Graphics
  SFML::Window
  SFML::System
  SFML::Audio
)
