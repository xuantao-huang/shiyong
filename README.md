# Diff-YOPO v1.1.1

## 概述

**Diff-YOPO** 是为**微分智飞**公司旗下教育无人机子品牌**非凸空间**适配的导航避障算法。Diff-YOPO 是在天津大学开源端到端规划器 YOPO（You Only Plan Once，IEEE RA-L 2024）基础上，针对 非凸-α 平台深度优化的自主飞行规划方案。

## 使用步骤

### 1. 下载源码并编译:

```
git clone --branch v1.1.1  https://gitee.com/DifferentialRobotics/Diff-YOPO.git
```

### 2. 编译代码

  ```
    cd ~/Diff-YOPO/ros_ws
    catkin_make
  ```

### 3. 导航飞行

  ```
    ./sh_files/run_all.sh
  ```

## 致谢与声明
本项目在开发过程中参考并使用了以下开源项目：
- **[YOPO](https://github.com/TJU-Aerial-Robotics/YOPO)**，特此感谢天津大学 **TJU-Aerial-Robotics** 团队的开源贡献。
- **[Faster-LIO](https://github.com/gaoxiang12/faster-lio)** ，特此感谢项目作者团队的开源贡献。
- **[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)** 驱动，特此感谢项目作者团队的开源贡献。

相关代码均严格遵循原项目的开源许可协议使用，用户在使用本项目时，请务必遵守相应的许可证条款。

# Q&A
请随时提交问题或讨论，我们会在看到问题后尽快回复。

