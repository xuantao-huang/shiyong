## 编译(根据报错安装完所有依赖直到编译通过)

```
cd /home/nv/YOPO-j30v2-Lidar/ros_ws
catkin_make
如果报错livox_ros_driver2相关，可以加入build.sh文件，使用以下指令完成编译：./build.sh ROS1
```

## 实机部署(6 终端)

> 所有终端先 `source /opt/ros/noetic/setup.zsh && source /home/nv/YOPO/ros_ws/devel/setup.zsh`

**T1 — MAVROS和雷达定位**

```
cd /home/nv/YOPO/ros_ws
./sh_files/run_lio.sh
```

**T2 — PX4控制器**

```bash
cd /home/nv/YOPO/ros_ws
source devel/setup.zsh
roslaunch px4ctrl run_ctrl_lio.launch
```

**T3 — 起飞**

```bash
cd /home/nv/YOPO/ros_ws
bash sh_files/takeoff.sh
```

`px4ctrl` 自动 OFFBOARD → arm → 爬升至悬停高度。

**T4 — YOPO**

```
cd /home/nv/YOPO/YOPO
python test_yopo_ros.py --use_tensorrt 1
```

**T5 — RViz**

```cd
rviz -d YOPO/yopo.rviz
```

**T6 — 降落**

```bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
```

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {seq: 0, stamp: now, frame_id: 'base_link'}, pose: {position: {x: 6.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
