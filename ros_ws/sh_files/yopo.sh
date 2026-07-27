#!/bin/zsh
source devel/setup.zsh;
cd /home/nv/Diff-YOPO/YOPO
source /opt/ros/noetic/setup.zsh && source /home/nv/Diff-YOPO/ros_ws/devel/setup.zsh
python test_yopo_ros.py --use_tensorrt 1 & sleep 5;