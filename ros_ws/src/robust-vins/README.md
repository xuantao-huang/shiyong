# Fast and Robust Visual Inertial Navigation System (FAR-VINS)

## User Tutorial

- [https://qhwukij0eg.feishu.cn/docx/XmO1dLG4Qo6vNTxXyI0cnjMonuc?from=from_copylink](https://qhwukij0eg.feishu.cn/docx/XmO1dLG4Qo6vNTxXyI0cnjMonuc?from=from_copylink)

## Quick Start in Three Minutes

- Build

```shell
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/weihaoysgs/FAR-VINS.git
cd ..
catkin_make -j
```

- Run on a real device (default)

```shell
source devel/setup.bash
export DRONE_ID=1
roslaunch far_vins bringup.launch
```

The default bringup launch starts the RealSense camera, MAVROS, FAR-VINS and RViz.

## Offline Playback

```shell
# run euroc dataset
source devel/setup.bash
roslaunch far_vins far_euroc_vio_stereo.launch
roslaunch far_vins rviz.launch
rosbag play V2_03_difficult.bag
```

```shell
# run FAST Lab dataset from bag
source devel/setup.bash
roslaunch far_vins far_fast_lab_3588_vio_stereo.launch
roslaunch far_vins rviz.launch
rosbag play xx.bag
```

## Parameter

vio system will read `camera intrinsics` and `DRONE_ID` from ENV in the fy project.

```shell
export fx=300
export fy=301
export cx=200
export cy=201
export DRONE_ID=1
```

## Evaluate Report

- [https://qhwukij0eg.feishu.cn/docx/AfZzdJmokop4bXxppa2czMDsnIe](https://qhwukij0eg.feishu.cn/docx/AfZzdJmokop4bXxppa2czMDsnIe)
- [https://qhwukij0eg.feishu.cn/docx/KlyndehFtoFnuBxeoUicCb6mnSb](https://qhwukij0eg.feishu.cn/docx/KlyndehFtoFnuBxeoUicCb6mnSb)

## TODO
- check angle error for mei camera model
- check `/// add` operation
- solve ENUM bug in `camera_geometry_base.h` 
