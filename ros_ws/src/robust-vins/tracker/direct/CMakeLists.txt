project(direct)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

string(ASCII 27 Esc)
set(R "${Esc}[4;32m") # set(R "${Esc}[7;32m")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "${R}aarch64 CPU${E} HAVE_FAST_NEON ENABLE NEON")
  ADD_DEFINITIONS(-DHAVE_FAST_NEON)
endif()

set(HEADERS
  patch_warp.h
  patch_score.h
  patch_utils.h
  feature_alignment.h
  feature_detection_types.h
  feature_detection.h
  feature_detection_utils.h
  matcher.h
  depth_filter.h
  depth_estimation.h
  elder_zucker.h
)

set(SOURCES
  internal/patch_warp.cpp
  internal/feature_alignment.cpp
  internal/feature_detection.cpp
  internal/feature_detection_utils.cpp
  internal/matcher.cpp
  internal/depth_filter.cpp
  internal/depth_estimation.cpp
  internal/elder_zucker.cpp
  )

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} fast)

# cs_add_executable(svo_test_patch_score test/test_patch_score.cpp)
# target_link_libraries(svo_test_patch_score ${PROJECT_NAME})

# cs_add_executable(svo_test_edgelet_direction test/test_edgelet_direction.cpp)
# target_link_libraries(svo_test_edgelet_direction ${PROJECT_NAME})

# cs_add_executable(svo_test_edgelet_extraction test/test_edgelet_extraction.cpp)
# target_link_libraries(svo_test_edgelet_extraction ${PROJECT_NAME})

# cs_add_executable(svo_test_feature_alignment test/test_feature_alignment.cpp)
# target_link_libraries(svo_test_feature_alignment ${PROJECT_NAME})

################################################################################
###
### GTEST
###

# catkin_add_gtest(${PROJECT_NAME}-test-depth-estimation
#   test/test_depth_estimation.cpp
#   test/test_utils.h
# )
# target_link_libraries(${PROJECT_NAME}-test-depth-estimation ${PROJECT_NAME} pthread)

