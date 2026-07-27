project(align)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(HEADERS_CPU
  sparse_img_align_base.h
  sparse_img_align.h
)

set(SOURCES_CPU
  internal/sparse_img_align_base.cpp
  internal/sparse_img_align.cpp
)

add_library(${PROJECT_NAME} ${SOURCES_CPU} ${HEADERS_CPU})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} vikit_solver)

