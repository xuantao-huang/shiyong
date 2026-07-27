#!/bin/zsh
source ros_ws/devel/setup.zsh;
# source /opt/ros/noetic/setup.zsh && source /home/nv/Diff-YOPO/ros_ws/devel/setup.zsh
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {seq: 0, stamp: now, frame_id: 'base_link'}, pose: {position: {x: 25, y: 5.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
sleep 20
# 返航：机身原点，向后飞回
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {seq: 0, stamp: now, frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" & sleep 5;
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
