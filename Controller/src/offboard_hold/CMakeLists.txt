cmake_minimum_required(VERSION 3.0.2)
project(offboard_hold)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs)

catkin_package()

# catkin_install_python(
#   PROGRAMS scripts/hold_position.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
