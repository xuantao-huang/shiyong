add_executable(test_include test_dependency.cpp)
target_link_libraries(test_include FAR_VINS)
add_executable(test_stereo_matcher test_stereo_matcher.cpp)
target_link_libraries(test_stereo_matcher FAR_VINS)
