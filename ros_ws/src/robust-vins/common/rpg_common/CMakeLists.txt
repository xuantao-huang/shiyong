project(rpg_common)
cmake_minimum_required(VERSION 2.8.3)

FILE(GLOB_RECURSE HEADERS "include/*")

list(APPEND SOURCES src/fs.cpp)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
