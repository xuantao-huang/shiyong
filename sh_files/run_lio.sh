#!/bin/zsh
echo 'nv' | sudo -S chmod 777 /dev/tty* & sleep 1;
roslaunch mavros px4.launch & sleep 6;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
source ros_ws/devel/setup.zsh;
roslaunch faster_lio mapping_mid360.launch & sleep 5;
roslaunch ekf ekf_lidar.launch & sleep 5;
# roslaunch diff_planner exp_rviz.launch & sleep 1;
roslaunch px4ctrl run_ctrl_lio.launch & sleep 5;

cd /home/nv/Diff-YOPO/YOPO
rviz -d Diff-YOPO/yopo.rviz & sleep 2;
wait;
