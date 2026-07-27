project(fast)
cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

string(ASCII 27 Esc)
set(R "${Esc}[4;32m") # set(R "${Esc}[7;32m")

message(STATUS "${R}System Host NAME: ${E}" ${CMAKE_HOST_SYSTEM_NAME})
message(STATUS "${R}HOST SYSTEM PROCESSOR Architecture: ${E}" ${CMAKE_HOST_SYSTEM_PROCESSOR})

if(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux")
  message(STATUS "${R}Linux Platform${E}")
elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Android")
	message(STATUS "${R}Android Platform${E}")
endif()
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "${R}aarch64 CPU${E} Open __ARM_NEON__")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
  # ADD_DEFINITIONS(-DHAVE_FAST_NEON)
  ADD_DEFINITIONS(-D__ARM_NEON__)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
	message(STATUS "${R}x86_64 CPU Open SSE${E}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
endif()

# IF(DEFINED ENV{ARM_ARCHITECTURE})
# 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a")
# ELSE()
# 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
# ENDIF()

set(HEADERS
  include/fast/corner_9.h
  include/fast/corner_10.h
  include/fast/fast.h
  include/fast/faster_corner_utilities.h
  )

set(SOURCES
  src/fast_9.cpp
  src/fast_10.cpp
  src/fast_10_score.cpp
  src/nonmax_3x3.cpp
  )

# IF(DEFINED ENV{ARM_ARCHITECTURE})
#   LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
# ELSE()
#   LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
# ENDIF()
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
endif()

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

