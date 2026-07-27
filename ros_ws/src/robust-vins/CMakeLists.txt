cmake_minimum_required(VERSION 2.8...3.22)
project(far_vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/backward-cpp)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "${R}aarch64 CPU${E} HAVE_FAST_NEON ENABLE NEON")
  ADD_DEFINITIONS(-DHAVE_FAST_NEON)
endif()

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        cv_bridge
        pcl_ros
        image_transport
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}
    thirdparty/minkindr/include
    thirdparty/vikit/vikit_cameras/include
    thirdparty/vikit/vikit_common/include
    thirdparty/vikit/vikit_solver/include
    thirdparty/fast_neon/include
    common/rpg_common/include
)

add_subdirectory(thirdparty/vikit)
add_subdirectory(thirdparty/fast_neon)
add_subdirectory(common/rpg_common)
# add_subdirectory(tracker/direct)
# add_subdirectory(tracker/align)
# add_subdirectory(tracker/tracker)

set(THIRD_PARTY_LIBS
    glog
    gflags
    ${OpenCV_LIBS}
    vikit_cameras
    vikit_common
    vikit_solver
    rpg_common
    fast
    ${catkin_LIBRARIES}
    # direct
    # align
    # tracker
)

add_library(FAR_VINS
    common/internal/frame.cpp
    common/internal/local_feature.cpp
    common/internal/point.cpp
    common/internal/output_helper.cpp
    estimator/internal/schur_vins.cpp
    estimator/internal/map.cpp
    estimator/internal/frame_handler_stereo.cpp
    estimator/internal/initialization.cpp
    estimator/internal/imu_handler.cpp
    estimator/internal/frame_handler_base.cpp
    estimator/internal/stereo_triangulation.cpp
    estimator/internal/visualizer.cpp
    estimator/internal/reprojector.cpp
    estimator/internal/far_factory.cpp
    estimator/internal/far_interface.cpp
    estimator/internal/far_nodelet.cpp
    estimator/internal/far_node_base.cpp

    tracker/tracker/internal/feature_tracker.cpp
    tracker/tracker/internal/feature_tracking_types.cpp
    tracker/tracker/internal/feature_tracking_utils.cpp
    tracker/tracker/internal/feature_tracking_viz.cpp

    tracker/direct/internal/depth_estimation.cpp
    tracker/direct/internal/depth_filter.cpp
    tracker/direct/internal/elder_zucker.cpp
    tracker/direct/internal/feature_alignment.cpp
    tracker/direct/internal/feature_detection.cpp
    tracker/direct/internal/feature_detection_utils.cpp
    tracker/direct/internal/matcher.cpp
    tracker/direct/internal/patch_warp.cpp

    tracker/align/internal/sparse_img_align_base.cpp
    tracker/align/internal/sparse_img_align.cpp
)

target_link_libraries(FAR_VINS ${THIRD_PARTY_LIBS})

add_subdirectory(test)

add_executable(FAR_vio estimator/internal/far_node.cpp ${BACKWARD_ENABLE})
target_link_libraries(FAR_vio FAR_VINS)
add_backward(FAR_vio)