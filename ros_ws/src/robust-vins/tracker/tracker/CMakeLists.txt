project(tracker)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(HEADERS
  feature_tracking_types.h
  feature_tracking_utils.h
  feature_tracking_viz.h
  feature_tracker.h
)

set(SOURCES
  internal/feature_tracking_types.cpp
  internal/feature_tracking_utils.cpp
  internal/feature_tracking_viz.cpp
  internal/feature_tracker.cpp
)

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})