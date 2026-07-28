#!/bin/zsh
echo 'nv' | sudo -S chmod 777 /dev/tty* & sleep 1;
source ros_ws/devel/setup.zsh
roslaunch mavros px4.launch &sleep 6
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1
roslaunch faster_lio mapping_mid360.launch &sleep 5
roslaunch ekf ekf_lidar.launch &sleep 5
roslaunch px4ctrl run_ctrl_lio.launch &sleep 5
cd YOPO
python test_yopo_ros.py --use_tensorrt 1 &sleep 8
python rc_multipoint_control.py &
echo ""
echo "  >>> Waiting for rc_multipoint_control readiness... <<<"
echo "  (Watch for 'ALL SYSTEMS READY' before operating RC)"
echo ""

wait
