import os, sys
import cv2
import time
import torch
import numpy as np
from torch.utils.data import Dataset, DataLoader
from scipy.spatial.transform import Rotation as R
from sklearn.model_selection import train_test_split
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from config.config import cfg


class YOPODataset(Dataset):
    def __init__(self, mode='train', val_ratio=0.1):
        super(YOPODataset, self).__init__()
        # image params
        self.height = int(cfg["image_height"])
        self.width = int(cfg["image_width"])

        # LiDAR range image params — must match simulator config and inference-side params
        self.lidar_H        = self.height   # range image rows = 96
        self.lidar_W        = 360
        self.lidar_elev_min = np.radians(cfg.get('lidar_elev_min_deg', -7.0))
        self.lidar_elev_max = np.radians(cfg.get('lidar_elev_max_deg', 52.0))
        self.lidar_max_dist = cfg.get('lidar_max_dist', 20.0)

        # Backprojection LUT: precompute sin/cos per row/col (same as inference _bp_* tables)
        _rows = np.arange(self.lidar_H, dtype=np.float32)
        _cols = np.arange(self.lidar_W, dtype=np.float32)
        _elev = self.lidar_elev_max - _rows / self.lidar_H * (self.lidar_elev_max - self.lidar_elev_min)
        _az   = _cols / self.lidar_W * 2 * np.pi - np.pi
        self._bp_cos_el = np.cos(_elev)   # (H,)
        self._bp_sin_el = np.sin(_elev)
        self._bp_cos_az = np.cos(_az)     # (W,)
        self._bp_sin_az = np.sin(_az)

        # Perspective camera intrinsics — identical to inference _persp_* computation
        _half_v             = (self.lidar_elev_max - self.lidar_elev_min) / 2
        self._persp_pitch   = (self.lidar_elev_max + self.lidar_elev_min) / 2
        self._persp_sin_p   = np.sin(self._persp_pitch)
        self._persp_cos_p   = np.cos(self._persp_pitch)
        self._persp_fy      = (self.height / 2) / np.tan(_half_v)
        self._persp_cy      = self.height / 2
        _h_fov              = np.radians(cfg['horizon_camera_fov'])
        self._persp_fx      = (self.width  / 2) / np.tan(_h_fov / 2)
        self._persp_cx      = self.width  / 2
        # ramdom state: x-direction: log-normal distribution, yz-direction: normal distribution
        self.vel_max = cfg["vel_max_train"]
        self.acc_max = cfg["acc_max_train"]
        self.vx_lognorm_mean = np.log(1 - cfg["vx_mean_unit"])
        self.vx_logmorm_sigma = np.log(cfg["vx_std_unit"])
        self.v_mean = np.array([cfg["vx_mean_unit"], cfg["vy_mean_unit"], cfg["vz_mean_unit"]])
        self.v_std = np.array([cfg["vx_std_unit"], cfg["vy_std_unit"], cfg["vz_std_unit"]])
        self.a_mean = np.array([cfg["ax_mean_unit"], cfg["ay_mean_unit"], cfg["az_mean_unit"]])
        self.a_std = np.array([cfg["ax_std_unit"], cfg["ay_std_unit"], cfg["az_std_unit"]])
        self.goal_length = cfg['goal_length']
        self.goal_pitch_std = cfg["goal_pitch_std"]
        self.goal_yaw_std = cfg["goal_yaw_std"]
        if mode == 'train': self.print_data()

        # dataset
        base_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(base_dir, "../", cfg["dataset_path"])
        self.img_list, self.map_idx, self.positions, self.quaternions = [], [], np.empty((0, 3), dtype=np.float32), np.empty((0, 4), dtype=np.float32)

        datafolders = [f.path for f in os.scandir(data_dir) if f.is_dir()]
        datafolders.sort(key=lambda x: int(os.path.basename(x)))
        if mode == 'train':
            print("Datafolders:")
            for folder in datafolders:
                print("    ", folder)

        print("Loading", mode, "dataset")
        for data_idx in range(len(datafolders)):
            datafolder = datafolders[data_idx]

            image_file_names = [datafolder + "/" + filename
                                for filename in os.listdir(datafolder)
                                if os.path.splitext(filename)[1] == '.png'
                                and os.path.basename(filename).startswith('range_')]
            image_file_names.sort(key=lambda x: int(os.path.basename(x).split('.')[0].split("_")[1]))  # sort by filename to align with the label

            states = np.loadtxt(data_dir + f"/pose-{data_idx}.csv", delimiter=',', skiprows=1).astype(np.float32)
            positions = states[:, 0:3]
            quaternions = states[:, 3:7]

            file_names_train, file_names_val, positions_train, positions_val, quaternions_train, quaternions_val = train_test_split(
                image_file_names, positions, quaternions, test_size=val_ratio, random_state=0)

            if mode == 'train':
                self.img_list.extend(file_names_train)
                self.positions = np.vstack((self.positions, positions_train.astype(np.float32)))
                self.quaternions = np.vstack((self.quaternions, quaternions_train.astype(np.float32)))
                self.map_idx.extend([data_idx] * len(file_names_train))
            elif mode == 'valid':
                self.img_list.extend(file_names_val)
                self.positions = np.vstack((self.positions, positions_val.astype(np.float32)))
                self.quaternions = np.vstack((self.quaternions, quaternions_val.astype(np.float32)))
                self.map_idx.extend([data_idx] * len(file_names_val))
            else:
                raise ValueError(f"Invalid mode {mode}. Choose from 'train', 'valid'.")

        print(f"=============== {mode.capitalize()} Data Summary ===============")
        print(f"{'Images'      :<12} | Count: {len(self.img_list):<3} |  Shape: {self.width},{self.height}")
        print(f"{'Positions'   :<12} | Count: {self.positions.shape[0]:<3} |  Shape: {self.positions.shape[1]}")
        print(f"{'Quaternions' :<12} | Count: {self.quaternions.shape[0]:<3} |  Shape: {self.quaternions.shape[1]}")
        print("==================================================")

    def __len__(self):
        return len(self.img_list)

    def __getitem__(self, item):
        # 1. Load range image (96×360, 16-bit PNG) and convert to meters
        range_raw = cv2.imread(self.img_list[item], -1).astype(np.float32)
        range_raw = range_raw * (self.lidar_max_dist / 65535.0)  # [0, max_dist] meters
        # Re-project: spherical range → FLU point cloud → pitch-aligned perspective depth
        # Mirrors inference _project_to_perspective_depth exactly (same intrinsics/pitch)
        depth_persp = self._range_to_perspective(range_raw)       # (H, W) meters
        image = np.expand_dims(depth_persp / self.lidar_max_dist, axis=0)  # [0, 1]

        # W: world frame; B/b: body frame
        # w: level with the ground but with the same orientation (yaw) as the body frame
        q_wxyz = self.quaternions[item, :]  # q: wxyz
        R_WB = R.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
        euler_angles = R_WB.as_euler('ZYX', degrees=False)  # [yaw(z) pitch(y) roll(x)]
        R_Bw = R.from_euler('ZYX', [0, euler_angles[1], euler_angles[2]], degrees=False).inv()

        # 2. get random vel, acc in the direction of the quadrotor
        vel_w, acc_w = self._get_random_state()
        vel_b, acc_b = R_Bw.apply(vel_w), R_Bw.apply(acc_w)

        # 3. generate random goal in front of the quadrotor
        goal_w = self._get_random_goal()
        goal_b = R_Bw.apply(goal_w)

        random_obs = np.hstack((vel_b, acc_b, goal_b)).astype(np.float32)
        rot_wb = R_WB.as_matrix().astype(np.float32)  # transform to rot_matrix in numpy is faster than using quat in pytorch
        # vel & acc & goal are in body frame, NWU, and no-normalization
        return image, self.positions[item], rot_wb, random_obs, self.map_idx[item]

    def _range_to_perspective(self, range_img):
        """
        球坐标范围图（H×W，单位 m）→ 俯仰对齐透视深度图（height×width，单位 m）。
        与推理侧 _project_to_perspective_depth 完全对齐：相同内参、相同 pitch、相同欧氏距离语义。

        步骤：
          1. 每个有效像素按球坐标 LUT 反投影为 FLU 体坐标系 3D 点
          2. 用 pitch 旋转矩阵投影到透视图像平面
          3. 多点命中同一像素时取最近距离（np.minimum.at）
        """
        valid  = range_img < self.lidar_max_dist * 0.999
        rows, cols = np.where(valid)
        r      = range_img[rows, cols]
        x = r * self._bp_cos_el[rows] * self._bp_cos_az[cols]   # FLU: forward
        y = r * self._bp_cos_el[rows] * self._bp_sin_az[cols]   # FLU: left
        z = r * self._bp_sin_el[rows]                            # FLU: up

        sp, cp = self._persp_sin_p, self._persp_cos_p
        zc = cp * x + sp * z                  # depth along pitch-aligned optical axis
        front = zc > 0.05
        x, y, z, r, zc = x[front], y[front], z[front], r[front], zc[front]

        u_f = self._persp_fx * (-y)              / zc + self._persp_cx
        v_f = self._persp_fy * (sp * x - cp * z) / zc + self._persp_cy
        u_i = np.round(u_f).astype(np.int32)
        v_i = np.round(v_f).astype(np.int32)

        in_img = (u_i >= 0) & (u_i < self.width) & (v_i >= 0) & (v_i < self.height)
        u_i, v_i, r = u_i[in_img], v_i[in_img], r[in_img]

        depth = np.full((self.height, self.width), self.lidar_max_dist, dtype=np.float32)
        np.minimum.at(depth, (v_i, u_i), r)
        return depth

    def _get_random_state(self):
        while True:
            vel = self.vel_max * (self.v_mean + self.v_std * np.random.randn(3))
            right_skewed_vx = -1
            while right_skewed_vx < 0:
                right_skewed_vx = self.vel_max * np.random.lognormal(mean=self.vx_lognorm_mean, sigma=self.vx_logmorm_sigma, size=None)
                right_skewed_vx = -right_skewed_vx + 1.2 * self.vel_max  # * 1.2 to ensure v_max can be sampled
            vel[0] = right_skewed_vx
            if np.linalg.norm(vel) < 1.2 * self.vel_max:  # avoid outliers
                break

        while True:
            acc = self.acc_max * (self.a_mean + self.a_std * np.random.randn(3))
            if np.linalg.norm(acc) < 1.2 * self.acc_max:  # avoid outliers
                break
        return vel, acc

    def _get_random_goal(self):
        goal_pitch_angle = np.random.normal(0.0, self.goal_pitch_std)
        goal_yaw_angle = np.random.normal(0.0, self.goal_yaw_std)
        goal_pitch_angle, goal_yaw_angle = np.radians(goal_pitch_angle), np.radians(goal_yaw_angle)
        goal_w_dir = np.array([np.cos(goal_yaw_angle) * np.cos(goal_pitch_angle),
                               np.sin(goal_yaw_angle) * np.cos(goal_pitch_angle), np.sin(goal_pitch_angle)])
        # 10% probability to generate a nearby goal (× goal_length is actual length)
        random_near = np.random.rand()
        if random_near < 0.1:
            goal_w_dir = random_near * 10 * goal_w_dir
        return self.goal_length * goal_w_dir

    def print_data(self):
        import scipy.stats as stats
        # 计算Vx 5% ~ 95% 区间
        p5 = self.vel_max * np.exp(stats.norm.ppf(0.05, loc=self.vx_lognorm_mean, scale=self.vx_logmorm_sigma))
        p95 = self.vel_max * np.exp(stats.norm.ppf(0.95, loc=self.vx_lognorm_mean, scale=self.vx_logmorm_sigma))

        v_lower = self.vel_max * (self.v_mean - 2 * self.v_std)
        v_upper = self.vel_max * (self.v_mean + 2 * self.v_std)
        v_lower[0] = max(-p95 + 1.2 * self.vel_max, 0)
        v_upper[0] = -p5 + 1.2 * self.vel_max

        a_lower = self.acc_max * (self.a_mean - 2 * self.a_std)
        a_upper = self.acc_max * (self.a_mean + 2 * self.a_std)

        print("----------------- Sampling State --------------------")
        print("| X-Y-Z | Vel 95% Range(m/s)  | Acc 95% Range(m/s2) |")
        print("|-------|---------------------|---------------------|")
        for i in range(3):
            print(f"|  {i:^4} | {v_lower[i]:^9.1f}~{v_upper[i]:^9.1f} |"
                  f" {a_lower[i]:^9.1f}~{a_upper[i]:^9.1f} |")
        print("-----------------------------------------------------")
        print(f"| Goal Pitch 90% (deg)        | {-self.goal_pitch_std * 2:^9.1f}~{self.goal_pitch_std * 2:^9.1f} |")
        print(f"| Goal Yaw   90% (deg)        | {-self.goal_yaw_std * 2:^9.1f}~{self.goal_yaw_std * 2:^9.1f} |")
        print("-----------------------------------------------------")

    def plot_sample_distribution(self):
        import matplotlib.pyplot as plt
        # ===== 采样 =====
        N = 10000
        goals = np.array([self._get_random_goal() for _ in range(N)])
        states = np.array([self._get_random_state() for _ in range(N)])
        vels = np.stack([s[0] for s in states])
        accs = np.stack([s[1] for s in states])

        x, y, z = goals[:, 0], goals[:, 1], goals[:, 2]
        yaw = np.degrees(np.arctan2(y, x))  # 水平角 [-180, 180]
        pitch = np.degrees(np.arctan2(z, np.sqrt(x ** 2 + y ** 2)))  # 垂直角 [-90, 90]

        fig, axs = plt.subplots(3, 3, figsize=(15, 10))

        # Goal方向角分布
        axs[0, 0].hist(yaw, bins=180)
        axs[0, 0].set_title("Goal Yaw Distribution")
        axs[0, 0].set_xlabel("Yaw (deg)")
        axs[0, 0].set_xlim([-60, 60])
        axs[0, 0].grid(True)

        axs[0, 1].hist(pitch, bins=90)
        axs[0, 1].set_title("Goal Pitch Distribution")
        axs[0, 1].set_xlabel("Pitch (deg)")
        axs[0, 1].set_xlim([-60, 60])
        axs[0, 1].grid(True)

        # Goal往图像投影分布(未考虑机体旋转)
        axs[0, 2].scatter(yaw, pitch, s=2, alpha=0.3)
        axs[0, 2].set_title("Goal Distribution in Image")
        axs[0, 2].set_xlabel("Yaw (deg)")
        axs[0, 2].set_ylabel("Pitch (deg)")
        axs[0, 2].set_xlim([-45, 45])
        axs[0, 2].set_ylim([-30, 30])
        axs[0, 2].grid(True)

        # Velocity分布
        for i, name in enumerate(['Vx', 'Vy', 'Vz']):
            axs[1, i].hist(vels[:, i], bins=100)
            axs[1, i].set_title(f"Velocity {name}")
            axs[1, i].grid(True)

        # Acceleration分布
        for i, name in enumerate(['Ax', 'Ay', 'Az']):
            axs[2, i].hist(accs[:, i], bins=100)
            axs[2, i].set_title(f"Acceleration {name}")
            axs[2, i].grid(True)

        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    # plot the random sample
    dataset = YOPODataset()
    dataset.plot_sample_distribution()

    # select the best num_workers
    max_workers = os.cpu_count()
    print(f"\n✅ cpu_count = {max_workers}")

    results = []
    for nw in range(0, max_workers + 1):
        data_loader = DataLoader(dataset, batch_size=16, shuffle=True, num_workers=nw)
        start = time.time()
        for i, _ in enumerate(data_loader):
            if i > 50:  # 只测前50个batch
                break
        torch.cuda.synchronize() if torch.cuda.is_available() else None
        elapsed = time.time() - start
        results.append((nw, elapsed))
        print(f"num_workers={nw}: {elapsed:.3f}s")

    best = min(results, key=lambda x: x[1])
    print(f"\n✅ 最优 num_workers = {best[0]}, 平均耗时={best[1]:.3f}s")
