cmake_minimum_required(VERSION 2.8.3)
project(vikit_solver)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(yaml-cpp)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  include/vikit/solver/mini_least_squares_solver.h
  include/vikit/solver/implementation/mini_least_squares_solver.hpp
  src/robust_cost.cpp
  include/vikit/solver/robust_cost.h
)

