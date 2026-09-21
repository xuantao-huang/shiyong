import rospy
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from threading import Lock, Thread, Event
from collections import deque
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs import point_cloud2

import cv2
import os
import time
import torch
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R

from config.config import cfg
from control_msg import PositionCommand
from policy.yopo_network import YopoNetwork
from policy.poly_solver import *
from policy.state_transform import *

try:
    from torch2trt import TRTModule
except ImportError:
    print("tensorrt not found.")


class YopoNet:
    def __init__(self, config, weight):
        self.config = config
        rospy.init_node('yopo_net', anonymous=False)
        # load params
        cfg["train"] = False
        self.height = cfg['image_height']
        self.width = cfg['image_width']
        self.min_dis, self.max_dis = 0.3, 20.0
        goal_cfg = self.config.get('goal')
        self.goal = np.array(goal_cfg) if goal_cfg is not None else None
        self.plan_from_reference = self.config['plan_from_reference']
        self.use_trt = self.config['use_tensorrt']
        self.verbose = self.config['verbose']
        self.visualize = self.config['visualize']
        self.Rotation_bc = R.from_euler('ZYX', [0, self.config['pitch_angle_deg'], 0], degrees=True).as_matrix()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        # camera intrinsics for depth point cloud visualization
        self.cam_fx = self.config.get('cam_fx', 161.0)
        self.cam_fy = self.config.get('cam_fy', 161.0)
        self.cam_cx = self.config.get('cam_cx', 80.0)
        self.cam_cy = self.config.get('cam_cy', 48.0)
        self.depth_vis_stride = self.config.get('depth_vis_stride', 4)  # subsample stride for performance

        # variables
        self.odom = Odometry()
        self.odom_init = False
        self.last_yaw = 0.0
        self.ctrl_dt = 0.02
        self.ctrl_time = None
        self.desire_init = False
        self.arrive = False
        self.desire_pos = None
        self.desire_vel = None
        self.desire_acc = None
        self.optimal_poly_x = None
        self.optimal_poly_y = None
        self.optimal_poly_z = None
        self.lock = Lock()
        self.last_control_msg = None
        self.state_transform = StateTransform()
        self.lattice_primitive = LatticePrimitive.get_instance()
        self.traj_time = self.lattice_primitive.segment_time

        # eval
        self.time_forward = 0.0
        self.time_process = 0.0
        self.time_prepare = 0.0
        self.time_interpolation = 0.0
        self.time_visualize = 0.0
        self.count = 0
        self.depth_fps = 30  # used only as processing time tolerance for printing logs

        # Load Network
        if self.use_trt:
            self.policy = TRTModule()
            self.policy.load_state_dict(torch.load(weight))
        else:
            state_dict = torch.load(weight, weights_only=True)
            self.policy = YopoNetwork()
            self.policy.load_state_dict(state_dict)
            self.policy = self.policy.to(self.device)
            self.policy.eval()
        self.warm_up()

        # ros publisher
        self.lattice_traj_pub = rospy.Publisher("/yopo_net/lattice_trajs_visual", PointCloud2, queue_size=1)
        self.best_traj_pub = rospy.Publisher("/yopo_net/best_traj_visual", PointCloud2, queue_size=1)
        self.all_trajs_pub = rospy.Publisher("/yopo_net/trajs_visual", PointCloud2, queue_size=1)
        self.depth_cloud_pub = rospy.Publisher("/yopo_net/depth_cloud_world", PointCloud2, queue_size=1)
        self.ctrl_pub = rospy.Publisher(self.config["ctrl_topic"], PositionCommand, queue_size=1)
        # lidar range image publishers（可视化用）
        self.range_image_pub        = rospy.Publisher("/yopo_net/range_image",         Image,       queue_size=1)
        self.range_image_inpaint_pub= rospy.Publisher("/yopo_net/range_image_inpaint", Image,       queue_size=1)
        self.range_coverage_pub     = rospy.Publisher("/yopo_net/range_coverage",      Image,       queue_size=1)
        self.range_cloud_pub        = rospy.Publisher("/yopo_net/range_cloud_body",    PointCloud2, queue_size=1)
        # 透视深度图发布器：显示实际喂给网络的 96×160 图像（去畸变后）
        self.persp_depth_pub        = rospy.Publisher("/yopo_net/persp_depth",         Image,       queue_size=1)
        # ros subscriber
        self.odom_sub = rospy.Subscriber(self.config['odom_topic'], Odometry, self.callback_odometry, queue_size=1, tcp_nodelay=True)
        self.depth_sub = rospy.Subscriber(self.config['depth_topic'], Image, self.callback_depth, queue_size=1, tcp_nodelay=True)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_set_goal, queue_size=1)

        # lidar 参数与状态
        self.lidar_H            = self.config.get('lidar_H', 96)
        self.lidar_W            = self.config.get('lidar_W', 360)
        self.lidar_elev_min     = np.radians(self.config.get('lidar_elev_min_deg',  -7.0))
        self.lidar_elev_max     = np.radians(self.config.get('lidar_elev_max_deg',  52.0))
        self.lidar_max_dist     = self.config.get('lidar_max_dist', 20.0)
        self.lidar_accum_frames  = self.config.get('lidar_accum_frames', 3)
        self._lidar_buf_lock     = Lock()
        self._lidar_world_buf    = deque(maxlen=self.lidar_accum_frames)
        self._lidar_latest_stamp = None
        self._lidar_new_data     = Event()
        # 预计算 backproject 用的 sin/cos 查找表，避免每帧重复trig运算
        _rows = np.arange(self.lidar_H, dtype=np.float32)
        _cols = np.arange(self.lidar_W, dtype=np.float32)
        _elev = (self.lidar_elev_max
                 - _rows / self.lidar_H * (self.lidar_elev_max - self.lidar_elev_min))
        _az   = _cols / self.lidar_W * 2 * np.pi - np.pi
        self._bp_cos_el, self._bp_sin_el = np.cos(_elev), np.sin(_elev)  # (H,)
        self._bp_cos_az, self._bp_sin_az = np.cos(_az),   np.sin(_az)    # (W,)
        # 透视深度图内参：
        #   垂直方向：自动从 LiDAR 实际仰角范围推算，使每一行都有有效点
        #     - 相机光轴仰角 = LiDAR 垂直 FOV 中心 (elev_min+elev_max)/2
        #     - fy = (H/2) / tan((elev_max-elev_min)/2)，图像恰好覆盖全部有效仰角
        #   水平方向：由 lidar_persp_hfov_deg 配置（默认训练值 90°，可放宽到 120°~150°）
        _half_v = (self.lidar_elev_max - self.lidar_elev_min) / 2
        self._persp_pitch   = (self.lidar_elev_max + self.lidar_elev_min) / 2  # 仰角，单位 rad
        self._persp_sin_p   = np.sin(self._persp_pitch)
        self._persp_cos_p   = np.cos(self._persp_pitch)
        self._persp_fy      = (self.height / 2) / np.tan(_half_v)
        self._persp_cy      = self.height / 2
        _h_fov = np.radians(self.config.get('lidar_persp_hfov_deg', cfg['horizon_camera_fov']))
        self._persp_fx = (self.width  / 2) / np.tan(_h_fov / 2)
        self._persp_cx = self.width  / 2
        rospy.loginfo(
            f"[LiDAR] perspective: pitch={np.degrees(self._persp_pitch):.1f}°  "
            f"H-FOV={np.degrees(_h_fov):.1f}°  "
            f"V-FOV={np.degrees(2*_half_v):.1f}° ({np.degrees(self.lidar_elev_min):.0f}°~{np.degrees(self.lidar_elev_max):.0f}°)  "
            f"fx={self._persp_fx:.1f}  fy={self._persp_fy:.1f}")
        # 虚拟天花板配置
        self.virtual_ceiling_enable = self.config.get('virtual_ceiling_enable', False)
        self.virtual_ceiling_z      = float(self.config.get('virtual_ceiling_z', 2.0))
        # 预计算每像素体系单位射线 LUT（用于虚拟天花板射线-平面交点法）
        _us = np.arange(self.width,  dtype=np.float32)
        _vs = np.arange(self.height, dtype=np.float32)
        _xc_n, _yc_n = np.meshgrid(
            (_us - self._persp_cx) / self._persp_fx,
            (_vs - self._persp_cy) / self._persp_fy)          # (H, W)
        sp, cp = self._persp_sin_p, self._persp_cos_p
        # R^T (cam→body FLU): x = sp*yc + cp*zc,  y = -xc,  z = sp*zc - cp*yc  (zc=1)
        _xb =  sp * _yc_n + cp
        _yb = -_xc_n
        _zb =  sp          - cp * _yc_n
        _norm = np.sqrt(_xb**2 + _yb**2 + _zb**2)
        self._ceil_ray_b_unit = np.stack(
            [_xb / _norm, _yb / _norm, _zb / _norm], axis=-1).astype(np.float32)  # (H, W, 3)
        rospy.loginfo(
            f"[VirtualCeiling] enable={self.virtual_ceiling_enable}  z={self.virtual_ceiling_z:.2f} m")
        # lidar subscriber（与 depth_sub 二选一激活，由 config['use_lidar'] 控制）
        if self.config.get('use_lidar', False):
            self.lidar_sub = rospy.Subscriber(
                self.config.get('lidar_topic', '/cloud_registered_body'),
                PointCloud2, self.callback_lidar, queue_size=1, tcp_nodelay=True)
            self._lidar_thread = Thread(target=self._lidar_inference_loop, daemon=True)
            self._lidar_thread.start()
        # ros timer
        rospy.sleep(1.0)  # wait connection...
        self.timer_ctrl = rospy.Timer(rospy.Duration(self.ctrl_dt), self.control_pub)
        print("YOPO Net Node Ready!")
        rospy.spin()

    def callback_set_goal(self, data):
        self.goal = np.asarray([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.arrive = False
        self.ctrl_time = None   # reset timer, wait for inference to generate new trajectory
        print(f"New Goal: ({data.pose.position.x:.1f}, {data.pose.position.y:.1f}, {self.goal[2]:.2f})")

    # the first frame
    def callback_odometry(self, data):
        # Skip messages with invalid (zero-norm) quaternion — happens during VIO initialization
        q = data.pose.pose.orientation
        if (q.x**2 + q.y**2 + q.z**2 + q.w**2) < 0.5:
            return
        self.odom = data
        if not self.desire_init:
            self.desire_pos = np.array((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z))
            self.desire_vel = np.array((self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z))
            self.desire_acc = np.array((0.0, 0.0, 0.0))
            ypr = R.from_quat([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                               self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]).as_euler('ZYX', degrees=False)
            self.last_yaw = ypr[0]
        self.odom_init = True

        if self.goal is not None:
            pos = np.array((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z))
            if np.linalg.norm(pos - self.goal) < 1 and not self.arrive:
                print("Arrive!")
                self.arrive = True

    def process_odom(self):
        # Rwb -> Rwc -> Rcw
        Rotation_wb = R.from_quat([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                   self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]).as_matrix()
        self.Rotation_wc = np.dot(Rotation_wb, self.Rotation_bc)
        Rotation_cw = self.Rotation_wc.T

        # vel and acc
        vel_w = self.desire_vel if self.plan_from_reference else np.array([self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z])
        vel_c = np.dot(Rotation_cw, vel_w)
        acc_w = self.desire_acc
        acc_c = np.dot(Rotation_cw, acc_w)

        # goal_dir: use actual odom position when plan_from_reference=False,
        # because desire_pos (reference trajectory) can advance far ahead of the
        # actual drone, causing goal_w to point backward once it overshoots the goal.
        if self.plan_from_reference:
            ref_pos = self.desire_pos
        else:
            ref_pos = np.array([self.odom.pose.pose.position.x,
                                self.odom.pose.pose.position.y,
                                self.odom.pose.pose.position.z])
        goal_w = self.goal - ref_pos
        goal_c = np.dot(Rotation_cw, goal_w)

        obs = np.concatenate((vel_c, acc_c, goal_c), axis=0).astype(np.float32)
        obs_norm = self.state_transform.normalize_obs(torch.from_numpy(obs[None, :]))
        return obs_norm

    # ------------------------------------------------------------------ #
    #  LiDAR 范围图处理流程                                               #
    # ------------------------------------------------------------------ #

    def _project_to_range_image(self, pts):
        """
        将体坐标系点云（FLU: X前 Y左 Z上，N×3 float32）投影为球坐标范围图。
        返回 (H, W) float32，无效像素填 lidar_max_dist。
        前向方向 = 中间列 (W//2)。
        """
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        dist = np.sqrt(x*x + y*y + z*z)

        valid = (dist > 0.1) & (dist < self.lidar_max_dist)
        x, y, z, dist = x[valid], y[valid], z[valid], dist[valid]

        azimuth   = np.arctan2(y, x)                        # [-π, π]
        elevation = np.arctan2(z, np.sqrt(x*x + y*y))      # [-π/2, π/2]

        in_fov = (elevation >= self.lidar_elev_min) & (elevation <= self.lidar_elev_max)
        azimuth, elevation, dist = azimuth[in_fov], elevation[in_fov], dist[in_fov]

        col = ((azimuth + np.pi) / (2 * np.pi) * self.lidar_W).astype(np.int32) % self.lidar_W
        row = ((self.lidar_elev_max - elevation) /
               (self.lidar_elev_max - self.lidar_elev_min) * self.lidar_H).astype(np.int32)
        col = np.clip(col, 0, self.lidar_W - 1)
        row = np.clip(row, 0, self.lidar_H - 1)

        range_img = np.full((self.lidar_H, self.lidar_W), self.lidar_max_dist, dtype=np.float32)
        np.minimum.at(range_img, (row, col), dist)
        return range_img

    def _project_to_perspective_depth(self, pts):
        """
        将体坐标系（FLU: X前 Y左 Z上）点云投影到仰角对齐 LiDAR FOV 的透视深度图 (H×W)。

        虚拟相机光轴仰角 = LiDAR 垂直 FOV 中心（默认 22.5°），使每行都有有效点。

        FLU → 仰角 pitch 的相机坐标系（R = Ry(pitch) @ R_default）：
          xc =  -y_body                           (右)
          yc =  sin_p * x_body - cos_p * z_body   (下，pitch 补偿后)
          zc =  cos_p * x_body + sin_p * z_body   (前，沿光轴)
        像素投影：
          u = fx * xc / zc + cx
          v = fy * yc / zc + cy
        深度值存储欧氏距离（与球坐标范围图保持一致）。
        """
        sp, cp = self._persp_sin_p, self._persp_cos_p
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        dist = np.sqrt(x*x + y*y + z*z)
        zc   = cp * x + sp * z                  # 沿光轴分量，>0 才在相机前方
        valid = (zc > 0.05) & (dist < self.lidar_max_dist)
        x, y, z, dist, zc = x[valid], y[valid], z[valid], dist[valid], zc[valid]
        if x.size == 0:
            return np.full((self.height, self.width), self.lidar_max_dist, dtype=np.float32)

        u = self._persp_fx * (-y)               / zc + self._persp_cx
        v = self._persp_fy * (sp * x - cp * z)  / zc + self._persp_cy
        u_i = np.round(u).astype(np.int32)
        v_i = np.round(v).astype(np.int32)

        in_fov = (u_i >= 0) & (u_i < self.width) & (v_i >= 0) & (v_i < self.height)
        u_i, v_i, dist = u_i[in_fov], v_i[in_fov], dist[in_fov]

        depth_img = np.full((self.height, self.width), self.lidar_max_dist, dtype=np.float32)
        np.minimum.at(depth_img, (v_i, u_i), dist)
        return depth_img

    def _backproject_perspective_depth(self, depth_img):
        """透视深度图（含 pitch）→ 体坐标系（FLU）3D 点云（N×3 float32）。
        R^T (cam→body): x = sp*yc + cp*zc,  y = -xc,  z = sp*zc - cp*yc
        """
        sp, cp = self._persp_sin_p, self._persp_cos_p
        valid = depth_img < self.lidar_max_dist * 0.999
        vs, us = np.where(valid)
        d = depth_img[vs, us]
        xc_n = (us - self._persp_cx) / self._persp_fx   # xc / zc
        yc_n = (vs - self._persp_cy) / self._persp_fy   # yc / zc
        scale = d / np.sqrt(1.0 + xc_n*xc_n + yc_n*yc_n)
        xc = xc_n * scale
        yc = yc_n * scale
        zc = scale
        x_body =  sp * yc + cp * zc
        y_body = -xc
        z_body =  sp * zc - cp * yc
        return np.stack([x_body, y_body, z_body], axis=1).astype(np.float32)

    def _inpaint_range_image(self, range_img):
        """形态学腐蚀填洞：空洞值=max_dist（最大），erode取局部最小将有效近距值扩散进空洞。"""
        invalid = range_img >= self.lidar_max_dist * 0.999
        result  = range_img.copy()
        kernel  = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        eroded  = cv2.erode(range_img, kernel, iterations=2)
        result[invalid] = eroded[invalid]
        return result

    def _apply_virtual_ceiling(self, depth_img, R_wb, pos_w):
        """在 inpaint 之后调用：仅对原本为 max_dist 的"空像素"叠加世界系 z=virtual_ceiling_z 平面的射线交点。

        必须 inpaint 之后调用：否则会用天花板距离覆盖 LiDAR 投影后尚未填洞的稀疏空像素，
        进而被 erode 扩散到整张图，把真实障碍淹没成散点。
        """
        z_ceiling = self.virtual_ceiling_z
        pos_z     = float(pos_w[2])
        t_floor   = max(self.min_dis * 1.1, self.min_dis + 0.05)  # 0.35 m，远离 invalid_net 阈值

        # 世界系单位射线：(H, W, 3) = ray_b_unit @ R_wb^T
        ray_w = self._ceil_ray_b_unit @ R_wb.T
        dz    = ray_w[..., 2]

        # 机体已超过天花板时 z_ceiling - pos_z ≤ 0，下面 t ≤ 0 会自动让 valid=False，不再叠加。
        t = (z_ceiling - pos_z) / np.where(dz > 1e-3, dz, np.inf)
        valid = (dz > 1e-3) & (t > 0) & (t < self.lidar_max_dist)

        # 几何截断：对每条朝上射线取 min(原始深度, 天花板交点距离)。
        # - 障碍 < 天花板：保留真实障碍（天花板下方的视野不受干扰）
        # - 障碍 > 天花板（或原本为 max_dist 的空像素）：替换为天花板距离，截断上方空间
        t_clipped = np.clip(t, t_floor, self.lidar_max_dist).astype(depth_img.dtype)
        return np.where(valid, np.minimum(depth_img, t_clipped), depth_img)

    def _make_range_image_msg(self, range_img, stamp, frame_id='body'):
        """将范围图编码为彩色 ROS Image（JET 色图，无效像素=黑色）。"""
        norm     = np.clip(range_img / self.lidar_max_dist, 0.0, 1.0)
        gray     = np.uint8((1.0 - norm) * 255)            # 近=亮，远=暗
        color    = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        color[range_img >= self.lidar_max_dist * 0.999] = 0  # 洞=黑
        color_rgb = color[:, :, ::-1].copy()               # BGR→RGB
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = color_rgb.shape[:2]
        msg.encoding = 'rgb8'
        msg.step     = msg.width * 3
        msg.data     = color_rgb.tobytes()
        return msg

    def _make_coverage_msg(self, range_img, stamp, frame_id='body'):
        """将有效像素掩码编码为 mono8 ROS Image（255=有效，0=无返回）。"""
        coverage = np.uint8((range_img < self.lidar_max_dist * 0.999) * 255)
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = coverage.shape
        msg.encoding = 'mono8'
        msg.step     = msg.width
        msg.data     = coverage.tobytes()
        return msg

    def _backproject_range_image(self, range_img):
        """将范围图反投影为体坐标系（FLU）3D 点云（N×3 float32）。用预计算LUT避免trig。"""
        valid = range_img < self.lidar_max_dist * 0.999
        rows, cols = np.where(valid)
        r      = range_img[rows, cols]
        cos_el = self._bp_cos_el[rows]
        sin_el = self._bp_sin_el[rows]
        cos_az = self._bp_cos_az[cols]
        sin_az = self._bp_sin_az[cols]
        x = r * cos_el * cos_az
        y = r * cos_el * sin_az
        z = r * sin_el
        return np.stack([x, y, z], axis=1).astype(np.float32)

    def callback_lidar(self, data):
        """LiDAR 回调：只做点云解析和世界系变换，<5ms，不会导致队列堆积。"""
        if not self.odom_init: return

        # 解析 PointCloud2 二进制（比 read_points 迭代器快 10-50x）
        raw = np.frombuffer(data.data, dtype=np.uint8).view(np.float32)
        step_floats = data.point_step // 4
        pts = raw.reshape(-1, step_floats)[:, :3].copy()
        valid_mask = np.isfinite(pts).all(axis=1)
        pts = pts[valid_mask]
        if pts.shape[0] == 0: return

        # 用当帧 odom 变换到世界系存入 buffer，推理线程再用最新 odom 反变换
        q = self.odom.pose.pose.orientation
        R_wb = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        pos_w = np.array([self.odom.pose.pose.position.x,
                          self.odom.pose.pose.position.y,
                          self.odom.pose.pose.position.z], dtype=np.float64)
        pts_w = pts @ R_wb.T + pos_w

        with self._lidar_buf_lock:
            self._lidar_world_buf.append(pts_w)
            self._lidar_latest_stamp = data.header.stamp

        self._lidar_new_data.set()

    @torch.inference_mode()
    def _lidar_inference_loop(self):
        """独立推理线程：从世界系 buffer 取点云，执行投影/inpaint/网络推理/发布控制。"""
        while not rospy.is_shutdown():
            if not self._lidar_new_data.wait(timeout=1.0):
                continue
            self._lidar_new_data.clear()
            if not self.odom_init: continue
            if self.goal is None or self.arrive: continue

            time0 = time.time()

            # 1. 快照 buffer（持锁时间极短）
            with self._lidar_buf_lock:
                if len(self._lidar_world_buf) == 0: continue
                all_pts_w = np.concatenate(list(self._lidar_world_buf), axis=0)
                stamp = self._lidar_latest_stamp

            # 用推理时刻的最新 odom 反变换，保证障碍物位置与当前机体姿态对齐
            q = self.odom.pose.pose.orientation
            R_wb_now = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            pos_w_now = np.array([self.odom.pose.pose.position.x,
                                  self.odom.pose.pose.position.y,
                                  self.odom.pose.pose.position.z], dtype=np.float64)
            all_pts = (all_pts_w - pos_w_now) @ R_wb_now

            # 2. 投影为 H×W 球坐标范围图（仅用于 360° 可视化）
            range_img_full = self._project_to_range_image(all_pts)
            range_img = self._inpaint_range_image(range_img_full)

            # 3. 直接投影到透视深度图（消除等距柱状图裁切后的 160° vs 90° FOV 畸变）
            #    先投影再 inpaint，确保在正确几何上填洞，避免畸变特征污染网络输入
            depth_persp = self._project_to_perspective_depth(all_pts)
            depth_persp = self._inpaint_range_image(depth_persp)
            if self.virtual_ceiling_enable:
                depth_persp = self._apply_virtual_ceiling(depth_persp, R_wb_now, pos_w_now)

            # 4. 发布可视化
            # range_image:        360° 原始球坐标图（参考用）
            # range_image_inpaint:96×160 透视深度图 inpaint 后（= 实际喂给网络的图像）
            # persp_depth:        同上（保留兼容旧订阅）
            # range_coverage:     360° 有效像素掩码
            # range_cloud_body:   360° inpaint 点云（体坐标系，参考用）
            if self.range_image_pub.get_num_connections() > 0:
                self.range_image_pub.publish(self._make_range_image_msg(range_img_full, stamp))
            if self.range_image_inpaint_pub.get_num_connections() > 0:
                self.range_image_inpaint_pub.publish(self._make_range_image_msg(depth_persp, stamp))
            if self.range_coverage_pub.get_num_connections() > 0:
                self.range_coverage_pub.publish(self._make_coverage_msg(range_img_full, stamp))
            if self.range_cloud_pub.get_num_connections() > 0:
                pts_body = self._backproject_range_image(range_img)
                header = std_msgs.msg.Header()
                header.stamp, header.frame_id = stamp, 'body'
                self.range_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts_body))
            if self.persp_depth_pub.get_num_connections() > 0:
                self.persp_depth_pub.publish(self._make_range_image_msg(depth_persp, stamp))

            time1 = time.time()

            # 5. 透视深度图已是 H×W（96×160），无需裁切
            crop = depth_persp

            # 6. 归一化
            depth_norm = np.minimum(crop, self.max_dis) / self.max_dis
            invalid_net = (depth_norm >= 0.999) | (depth_norm < self.min_dis / self.max_dis)
            if invalid_net.any():
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                filled = cv2.erode(depth_norm, kernel, iterations=1)
                depth_norm[invalid_net] = filled[invalid_net]
            depth = depth_norm.reshape(1, 1, self.height, self.width)

            time2 = time.time()

            # 7. 网络推理
            depth_input = torch.from_numpy(depth).to(self.device, non_blocking=True)
            obs_norm    = self.process_odom().to(self.device, non_blocking=True)
            obs_input   = self.state_transform.prepare_input(obs_norm)
            endstate_pred, score_pred = self.policy(depth_input, obs_input)
            endstate_pred, score_pred = endstate_pred.cpu().numpy(), score_pred.cpu().numpy()

            time3 = time.time()

            # 8. 后处理
            endstate, score = self.process_output(endstate_pred, score_pred,
                                                  return_all_preds=self.visualize)
            endstate_c = endstate.reshape(-1, 3, 3).transpose(0, 2, 1)
            endstate_w = np.matmul(self.Rotation_wc, endstate_c)

            action_id = np.argmin(score) if self.visualize else 0
            with self.lock:
                start_pos = (self.desire_pos if self.plan_from_reference
                             else np.array((self.odom.pose.pose.position.x,
                                            self.odom.pose.pose.position.y,
                                            self.odom.pose.pose.position.z)))
                start_vel = (self.desire_vel if self.plan_from_reference
                             else np.array((self.odom.twist.twist.linear.x,
                                            self.odom.twist.twist.linear.y,
                                            self.odom.twist.twist.linear.z)))
                self.optimal_poly_x = Poly5Solver(
                    start_pos[0], start_vel[0], self.desire_acc[0],
                    endstate_w[action_id, 0, 0] + start_pos[0],
                    endstate_w[action_id, 0, 1], endstate_w[action_id, 0, 2], self.traj_time)
                self.optimal_poly_y = Poly5Solver(
                    start_pos[1], start_vel[1], self.desire_acc[1],
                    endstate_w[action_id, 1, 0] + start_pos[1],
                    endstate_w[action_id, 1, 1], endstate_w[action_id, 1, 2], self.traj_time)
                self.optimal_poly_z = Poly5Solver(
                    start_pos[2], start_vel[2], self.desire_acc[2],
                    endstate_w[action_id, 2, 0] + start_pos[2],
                    endstate_w[action_id, 2, 1], endstate_w[action_id, 2, 2], self.traj_time)
                self.ctrl_time = 0.0

            time4 = time.time()
            self.visualize_trajectory(score_pred, endstate_w)
            if self.depth_cloud_pub.get_num_connections() > 0:
                # 用透视深度图反投影，显示网络实际输入对应的世界系点云
                pts_body = self._backproject_perspective_depth(depth_persp)
                pts_w = pts_body @ R_wb_now.T + pos_w_now
                header = std_msgs.msg.Header()
                header.stamp = stamp
                header.frame_id = 'world'
                self.depth_cloud_pub.publish(
                    point_cloud2.create_cloud_xyz32(header, pts_w.astype(np.float32)))
            time5 = time.time()
            self.print_time(time0, time1, time2, time3, time4, time5)

    @torch.inference_mode()
    def callback_depth(self, data):
        if not self.odom_init: return
        if self.goal is None or self.arrive: return

        # 1. Depth Image Process (Be careful with the depth units in your application)
        time0 = time.time()
        if data.encoding == "32FC1":    # Simulator, meter
            depth = np.frombuffer(data.data, dtype=np.float32).reshape(data.height, data.width)
        elif data.encoding == "16UC1":  # RealSense, millimeter
            depth = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width).astype(np.float32) / 1000.0
        else:
            raise ValueError(f"Unsupported depth encoding: {data.encoding}. Expected '32FC1' or '16UC1'.")

        if depth.shape[0] != self.height or depth.shape[1] != self.width:
            depth = cv2.resize(depth, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        depth = np.minimum(depth, self.max_dis) / self.max_dis

        # interpolated the nan value (experiment shows that treating nan directly as 0 produces similar results)
        nan_mask = np.isnan(depth) | (depth < self.min_dis / self.max_dis)
        interpolated_image = cv2.inpaint(np.uint8(depth * 255), np.uint8(nan_mask), 1, cv2.INPAINT_TELEA)
        interpolated_image = interpolated_image.astype(np.float32) / 255.0
        depth = interpolated_image.reshape([1, 1, self.height, self.width])
        # cv2.imshow("1", depth[0][0])
        # cv2.waitKey(1)

        # 2. YOPO Network Inference
        # input prepare
        time1 = time.time()
        depth_input = torch.from_numpy(depth).to(self.device, non_blocking=True)  # (non_blocking: copying speed 3x)
        obs_norm = self.process_odom().to(self.device, non_blocking=True)
        obs_input = self.state_transform.prepare_input(obs_norm)
        # torch.cuda.synchronize()

        time2 = time.time()
        # Forward (TensorRT: inference speed increased by 5x)
        endstate_pred, score_pred = self.policy(depth_input, obs_input)
        endstate_pred, score_pred = endstate_pred.cpu().numpy(), score_pred.cpu().numpy()
        time3 = time.time()

        # 3. Post-Processing
        # Replacing PyTorch operation on CUDA with NumPy operation on CPU (speed increased by 10x)
        endstate, score = self.process_output(endstate_pred, score_pred, return_all_preds=self.visualize)
        # Vectorization: transform the prediction(P V A in body frame) to the world frame with the attitude (without the position)
        endstate_c = endstate.reshape(-1, 3, 3).transpose(0, 2, 1)  # [N, 9] -> [N, 3, 3] -> [px vx ax, py vy ay, pz vz az]
        endstate_w = np.matmul(self.Rotation_wc, endstate_c)

        action_id = np.argmin(score) if self.visualize else 0
        with self.lock:  # Python3.8: threads are scheduled using time slices, add the lock to ensure safety
            start_pos = self.desire_pos if self.plan_from_reference else np.array((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z))
            start_vel = self.desire_vel if self.plan_from_reference else np.array((self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z))
            self.optimal_poly_x = Poly5Solver(start_pos[0], start_vel[0], self.desire_acc[0], endstate_w[action_id, 0, 0] + start_pos[0],
                                              endstate_w[action_id, 0, 1], endstate_w[action_id, 0, 2], self.traj_time)
            self.optimal_poly_y = Poly5Solver(start_pos[1], start_vel[1], self.desire_acc[1], endstate_w[action_id, 1, 0] + start_pos[1],
                                              endstate_w[action_id, 1, 1], endstate_w[action_id, 1, 2], self.traj_time)
            self.optimal_poly_z = Poly5Solver(start_pos[2], start_vel[2], self.desire_acc[2], endstate_w[action_id, 2, 0] + start_pos[2],
                                              endstate_w[action_id, 2, 1], endstate_w[action_id, 2, 2], self.traj_time)
            self.ctrl_time = 0.0
        time4 = time.time()
        self.visualize_trajectory(score_pred, endstate_w)
        self.publish_depth_cloud_world(depth[0, 0], data.header.stamp)
        time5 = time.time()

        self.print_time(time0, time1, time2, time3, time4, time5)

    def control_pub(self, _timer):
        if self.ctrl_time is None or self.ctrl_time > self.traj_time:
            return
        if self.arrive:
            if self.last_control_msg is not None:
                self.desire_init = False
                self.last_control_msg.trajectory_flag = self.last_control_msg.TRAJECTORY_STATUS_EMPTY
                self.ctrl_pub.publish(self.last_control_msg)
                self.last_control_msg = None
                rospy.loginfo("Goal reached, stopped control. px4ctrl will return to AUTO_HOVER.")
            self.ctrl_time = None
            return

        with self.lock:  # Python3.8: threads are scheduled using time slices, add the lock to ensure safety and publish frequency
            self.ctrl_time += self.ctrl_dt
            control_msg = PositionCommand()
            control_msg.header.stamp = rospy.Time.now()
            control_msg.trajectory_flag = control_msg.TRAJECTORY_STATUS_READY
            control_msg.position.x = self.optimal_poly_x.get_position(self.ctrl_time)
            control_msg.position.y = self.optimal_poly_y.get_position(self.ctrl_time)
            control_msg.position.z = self.optimal_poly_z.get_position(self.ctrl_time)
            control_msg.velocity.x = self.optimal_poly_x.get_velocity(self.ctrl_time)
            control_msg.velocity.y = self.optimal_poly_y.get_velocity(self.ctrl_time)
            control_msg.velocity.z = self.optimal_poly_z.get_velocity(self.ctrl_time)
            control_msg.acceleration.x = self.optimal_poly_x.get_acceleration(self.ctrl_time)
            control_msg.acceleration.y = self.optimal_poly_y.get_acceleration(self.ctrl_time)
            control_msg.acceleration.z = self.optimal_poly_z.get_acceleration(self.ctrl_time)
            control_msg.jerk.x = self.optimal_poly_x.get_jerk(self.ctrl_time)
            control_msg.jerk.y = self.optimal_poly_y.get_jerk(self.ctrl_time)
            control_msg.jerk.z = self.optimal_poly_z.get_jerk(self.ctrl_time)
            self.desire_pos = np.array([control_msg.position.x, control_msg.position.y, control_msg.position.z])
            self.desire_vel = np.array([control_msg.velocity.x, control_msg.velocity.y, control_msg.velocity.z])
            self.desire_acc = np.array([control_msg.acceleration.x, control_msg.acceleration.y, control_msg.acceleration.z])
            goal_dir = self.goal - self.desire_pos
            yaw, yaw_dot = calculate_yaw(self.desire_vel, goal_dir, self.last_yaw, self.ctrl_dt)
            self.last_yaw = yaw
            control_msg.yaw = yaw
            control_msg.yaw_dot = yaw_dot
            self.desire_init = True
            self.last_control_msg = control_msg
            self.ctrl_pub.publish(control_msg)

    def process_output(self, endstate_pred, score_pred, return_all_preds=False):
        endstate_pred = endstate_pred.reshape(9, self.lattice_primitive.traj_num).T
        score_pred = score_pred.reshape(self.lattice_primitive.traj_num)

        if not return_all_preds:
            action_id = np.argmin(score_pred)
            lattice_id = self.lattice_primitive.traj_num - 1 - action_id
            endstate = self.state_transform.pred_to_endstate_cpu(endstate_pred[action_id, :][np.newaxis, :], lattice_id)
            score = score_pred[action_id]
        else:
            score = score_pred
            endstate = self.state_transform.pred_to_endstate_cpu(endstate_pred, torch.arange(self.lattice_primitive.traj_num-1, -1, -1))

        return endstate, score

    def publish_depth_cloud_world(self, depth_norm, stamp):
        """Convert depth image to point cloud in world frame and publish."""
        if self.depth_cloud_pub.get_num_connections() == 0:
            return
        s = self.depth_vis_stride
        h, w = depth_norm.shape
        # subsampled pixel grid in camera frame
        us = np.arange(0, w, s)
        vs = np.arange(0, h, s)
        uu, vv = np.meshgrid(us, vs)  # (H', W')
        d = depth_norm[vv, uu] * self.max_dis  # restore metric depth (m)
        valid = (d > self.min_dis) & (d < self.max_dis)
        d = d[valid]
        uu = uu[valid].astype(np.float32)
        vv = vv[valid].astype(np.float32)
        # back-project to OpenCV camera frame (Z forward, X right, Y down)
        x_cv = (uu - self.cam_cx) / self.cam_fx * d
        y_cv = (vv - self.cam_cy) / self.cam_fy * d
        z_cv = d
        # convert to FLU body/camera frame (X forward, Y left, Z up) used by the codebase
        pts_c = np.stack([z_cv, -x_cv, -y_cv], axis=1)  # (N, 3)
        # transform to world frame: p_w = R_wc @ p_c + pos_w
        pos_w = np.array([self.odom.pose.pose.position.x,
                          self.odom.pose.pose.position.y,
                          self.odom.pose.pose.position.z])
        pts_w = pts_c @ self.Rotation_wc.T + pos_w  # (N, 3)
        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = 'world'
        self.depth_cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts_w.astype(np.float32)))

    def visualize_trajectory(self, pred_score, pred_endstate):
        dt = self.traj_time / 20.0
        start_pos = self.desire_pos if self.plan_from_reference else np.array((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z))
        start_vel = self.desire_vel if self.plan_from_reference else np.array((self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z))
        # best predicted trajectory
        if self.best_traj_pub.get_num_connections() > 0:
            t_values = np.arange(0, self.traj_time, dt)
            points_array = np.stack((
                self.optimal_poly_x.get_position(t_values),
                self.optimal_poly_y.get_position(t_values),
                self.optimal_poly_z.get_position(t_values)
            ), axis=-1)
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points_array)
            self.best_traj_pub.publish(point_cloud_msg)
        # lattice primitive
        if self.visualize and self.lattice_traj_pub.get_num_connections() > 0:
            lattice_endstate = self.lattice_primitive.lattice_pos_node.cpu().numpy()
            lattice_endstate = np.dot(lattice_endstate, self.Rotation_wc.T)
            zero_state = np.zeros_like(lattice_endstate)
            lattice_poly_x = Polys5Solver(start_pos[0], start_vel[0], self.desire_acc[0],
                                          lattice_endstate[:, 0] + start_pos[0], zero_state[:, 0], zero_state[:, 0], self.traj_time)
            lattice_poly_y = Polys5Solver(start_pos[1], start_vel[1], self.desire_acc[1],
                                          lattice_endstate[:, 1] + start_pos[1], zero_state[:, 1], zero_state[:, 1], self.traj_time)
            lattice_poly_z = Polys5Solver(start_pos[2], start_vel[2], self.desire_acc[2],
                                          lattice_endstate[:, 2] + start_pos[2], zero_state[:, 2], zero_state[:, 2], self.traj_time)
            t_values = np.arange(0, self.traj_time, dt)
            points_array = np.stack((
                lattice_poly_x.get_position(t_values),
                lattice_poly_y.get_position(t_values),
                lattice_poly_z.get_position(t_values)
            ), axis=-1)
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points_array)
            self.lattice_traj_pub.publish(point_cloud_msg)
        # all predicted trajectories
        if self.visualize and self.all_trajs_pub.get_num_connections() > 0:
            all_poly_x = Polys5Solver(start_pos[0], start_vel[0], self.desire_acc[0],
                                      pred_endstate[:, 0, 0] + start_pos[0], pred_endstate[:, 0, 1], pred_endstate[:, 0, 2], self.traj_time)
            all_poly_y = Polys5Solver(start_pos[1], start_vel[1], self.desire_acc[1],
                                      pred_endstate[:, 1, 0] + start_pos[1], pred_endstate[:, 1, 1], pred_endstate[:, 1, 2], self.traj_time)
            all_poly_z = Polys5Solver(start_pos[2], start_vel[2], self.desire_acc[2],
                                      pred_endstate[:, 2, 0] + start_pos[2], pred_endstate[:, 2, 1], pred_endstate[:, 2, 2], self.traj_time)
            t_values = np.arange(0, self.traj_time, dt)
            points_array = np.stack((
                all_poly_x.get_position(t_values),
                all_poly_y.get_position(t_values),
                all_poly_z.get_position(t_values)
            ), axis=-1)
            scores = np.repeat(pred_score, t_values.size)
            points_array = np.column_stack((points_array, scores))
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.FLOAT32, 1)]
            point_cloud_msg = point_cloud2.create_cloud(header, fields, points_array)
            self.all_trajs_pub.publish(point_cloud_msg)

    def print_time(self, time0, time1, time2, time3, time4, time5):
        """
        Performance reference: PyTorch model should take < 5 ms; TensorRT model should take < 1 ms

        Notes:
        - Running program and enabling RViz under WSL greatly increase processing time, and Ubuntu does not have these issues
        - Even with queue_size=1, it may cause message accumulation and lag when processing time exceeds the image frequency
        """
        self.time_interpolation = self.time_interpolation + (time1 - time0)
        self.time_prepare = self.time_prepare + (time2 - time1)
        self.time_forward = self.time_forward + (time3 - time2)
        self.time_process = self.time_process + (time4 - time3)
        self.time_visualize = self.time_visualize + (time5 - time4)
        self.count = self.count + 1

        total_time = (time5 - time0) * 1000
        tolerance = 1000.0 / self.depth_fps
        if total_time > tolerance:
            rospy.logwarn(f"Warn: Processing time {(time5 - time0) * 1000:.2f} ms exceeds {tolerance:.2f} ms, may cause message lag!")
            print(f"\033[34mCurrent Time Consuming:\033[0m "
                  f"depth-interpolation: \033[32m{1000 * (time1 - time0):.2f} ms\033[0m; "
                  f"data-prepare: \033[32m{1000 * (time2 - time1):.2f} ms\033[0m; "
                  f"network-inference: \033[32m{1000 * (time3 - time2):.2f} ms\033[0m; "
                  f"post-process: \033[32m{1000 * (time4 - time3):.2f} ms\033[0m; "
                  f"visualize-trajectory: \033[32m{1000 * (time5 - time4):.2f} ms\033[0m")
        if self.verbose or (total_time > tolerance):
            print(f"\033[34mAverage Time Consuming:\033[0m "
                  f"depth-interpolation: \033[32m{1000 * self.time_interpolation / self.count:.2f} ms\033[0m; "
                  f"data-prepare: \033[32m{1000 * self.time_prepare / self.count:.2f} ms\033[0m; "
                  f"network-inference: \033[32m{1000 * self.time_forward / self.count:.2f} ms\033[0m; "
                  f"post-process: \033[32m{1000 * self.time_process / self.count:.2f} ms\033[0m; "
                  f"visualize-trajectory: \033[32m{1000 * self.time_visualize / self.count:.2f} ms\033[0m")

    def warm_up(self):
        depth = torch.zeros((1, 1, self.height, self.width), dtype=torch.float32, device=self.device)
        obs = torch.zeros((1, 9), dtype=torch.float32, device=self.device)
        obs = self.state_transform.prepare_input(obs)
        endstate_pred, score_pred = self.policy(depth, obs)
        _ = self.state_transform.pred_to_endstate(endstate_pred)


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_tensorrt", type=int, default=0, help="use tensorrt or not")
    parser.add_argument("--trial", type=int, default=1, help="trial number")
    parser.add_argument("--epoch", type=int, default=50, help="epoch number")
    return parser


if __name__ == "__main__":
    args = parser().parse_args()
    base_dir = os.path.dirname(os.path.abspath(__file__))
    weight = "yopo_trt.pth" if args.use_tensorrt else base_dir + "/saved/YOPO_{}/epoch{}.pth".format(args.trial, args.epoch)
    print("load weight from:", weight)

    settings = {'use_tensorrt': args.use_tensorrt,
                # goal is set dynamically via /move_base_simple/goal (published by rc_multipoint_control)
                'pitch_angle_deg': -0,   # 相机俯仰角(仰为负)
                'odom_topic': '/ekf/ekf_odom',               # 里程计话题（实机）
                # 'odom_topic': '/sim/odom',                           # 里程计话题（仿真）
                'depth_topic': '/camera/depth/image_rect_raw',      # 深度图话题（use_lidar=False 时生效）
                'ctrl_topic': '/setpoints_cmd',               # 控制器话题
                'plan_from_reference': True,
                'verbose': False,               # 打印耗时？
                'visualize': True,           # 可视化所有轨迹？(实飞改为False节省计算)
                # camera intrinsics for depth point cloud (world frame) visualization
                'cam_fx': 98.92,
                'cam_fy': 79.14,
                'cam_cx': 81.0,
                'cam_cy': 47.53,
                'depth_vis_stride': 4,
                # ---- LiDAR 输入配置 ----
                'use_lidar': True,                      # True: 启用 lidar 推理；False: 使用深度相机
                'lidar_topic': '/laserMapping/cloud_registered_body',# FAST_LIO 体坐标系点云（已运动补偿）（实机）
                # 'lidar_topic': '/lidar_points',         # 仿真点云话题
                'lidar_H': 96,                          # 范围图高度（与 image_height 对齐）
                'lidar_W': 360,                         # 范围图宽度（360°，1°/列）
                'lidar_elev_min_deg': -7.0,             # 垂直 FOV 下限（MID-360 实际范围 -7°）
                'lidar_elev_max_deg':  52.0,            # 垂直 FOV 上限（MID-360 实际范围 +52°）
                'lidar_max_dist': 20.0,                 # 最大测距（m），与训练数据对齐
                'lidar_accum_frames': 2,                # 累积帧数（单帧，低延迟）
                # ---- 透视深度图水平 FOV（垂直方向自动对齐 LiDAR 仰角范围，无需配置）----
                # 训练值 90°，保证障碍物感知尺度与训练一致（安全距离正确）
                # 如需更宽覆盖可调大，但会导致障碍物像素变小，安全距离估计系统性偏远
                'lidar_persp_hfov_deg': 90.0,
                # ---- 虚拟天花板（生效值由 config/config.yaml 控制）----
                'virtual_ceiling_enable': cfg.get('virtual_ceiling_enable', True),
                'virtual_ceiling_z': cfg.get('virtual_ceiling_z', 2.6),
                }
    YopoNet(settings, weight)
