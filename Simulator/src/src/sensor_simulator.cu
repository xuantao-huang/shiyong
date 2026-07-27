#include "sensor_simulator.cuh"

namespace raycast
{   
    GridMap::GridMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution, int occupy_threshold = 1){
        const float epsilon = 0.001f;   // 避免数值误差导致 (1)建图空行 (2)边缘点被忽略
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        float length = max_pt(0) - min_pt(0) + 2 * epsilon;  // 保证各个边界最大值能被取到
        float width  = max_pt(1) - min_pt(1) + 2 * epsilon;
        float height = max_pt(2) - min_pt(2) + 2 * epsilon;
        Vector3f origin(min_pt(0), min_pt(1), min_pt(2));
        Vector3f map_size(length, width, height);
        origin_x_ = origin.x;
        origin_y_ = origin.y;
        origin_z_ = origin.z;

        Vector3i grid_size;
        grid_size.x = ceil(map_size.x / resolution);
        grid_size.y = ceil(map_size.y / resolution);
        grid_size.z = ceil(map_size.z / resolution);
        int grid_total_size = grid_size.x * grid_size.y * grid_size.z;

        resolution_   = resolution;
        grid_size_x_  = grid_size.x, 
        grid_size_y_  = grid_size.y, 
        grid_size_z_  = grid_size.z, 
        grid_size_yz_ = grid_size.y * grid_size.z;
        occupy_threshold_ = occupy_threshold;
        raycast_step_ = resolution;

        std::vector<int> h_map(grid_total_size, 0);
        // 点云全位于体素边界，有时候会有全空的行，加个很小的偏移
        for (size_t i = 0; i < cloud->points.size(); i++) {
            Vector3f point(cloud->points[i].x + epsilon, cloud->points[i].y + epsilon, cloud->points[i].z + epsilon);
            int idx = Vox2Idx(Pos2Vox(point));
            if (idx < grid_total_size) {
                h_map[idx]++;
            }
        }
        cudaMalloc((void **)&map_cuda_, grid_total_size * sizeof(int));
        cudaMemcpy(map_cuda_, h_map.data(), grid_total_size * sizeof(int), cudaMemcpyHostToDevice);
    }

    __host__ __device__ Vector3i GridMap::Pos2Vox(const Vector3f &pos)
    {
        Vector3i vox;
        vox.x = floor((pos.x - origin_x_) / resolution_);
        vox.y = floor((pos.y - origin_y_) / resolution_);
        vox.z = floor((pos.z - origin_z_) / resolution_);
        return vox;
    }

    __host__ __device__ Vector3f GridMap::Vox2Pos(const Vector3i &vox)
    {
        Vector3f pos;
        pos.x = (vox.x + 0.5f) * resolution_ + origin_x_;
        pos.y = (vox.y + 0.5f) * resolution_ + origin_y_;
        pos.z = (vox.z + 0.5f) * resolution_ + origin_z_;
        return pos;
    }

    __host__ __device__ int GridMap::Vox2Idx(const Vector3i &vox)
    {
        return vox.x * grid_size_yz_ + vox.y * grid_size_z_ + vox.z;
    }

    __host__ __device__ Vector3i GridMap::Idx2Vox(int idx)
    {
        return Vector3i(idx / grid_size_yz_, (idx % grid_size_yz_) / grid_size_z_, idx % grid_size_z_);
    }

    __device__ int GridMap::symmetricIndex(int index, int length)
    {
        index = index % (2 * length - 2);
        if (index < 0)
        {
            index += (2 * length - 2);
        }

        if (index >= length)
        {
            index = 2 * length - 2 - index;
        }
        return index;
    }

    // 0: 空闲; 1: 占据
    __device__  int GridMap::mapQuery(const Vector3f &pos){
        Vector3i vox = Pos2Vox(pos);
        vox.x = symmetricIndex(vox.x, grid_size_x_);
        vox.y = symmetricIndex(vox.y, grid_size_y_);

        if (vox.z >= grid_size_z_ || vox.z < 0)
            return 0;

        int idx = Vox2Idx(vox);
        if (map_cuda_[idx] > occupy_threshold_)
            return 1;
        return 0;
    }

    __global__ void cameraRaycastKernel(float* depth_values, GridMap grid_map, CameraParams camera_param, cudaMat::SE3<float> T_wc)
    {
        int u = threadIdx.x;
        int v = blockIdx.x;

        // printf("u: %d, v: %d \n", u, v);

        if (u < camera_param.image_width && v < camera_param.image_height)
        {
            // 计算射线方向
            float y = -(u - camera_param.cx) / camera_param.fx;
            float z = -(v - camera_param.cy) / camera_param.fy;
            float x = 1.0f;

            // 归一化射线方向
            float length = sqrtf(x * x + y * y + z * z);
            x /= length;
            y /= length;
            z /= length;

            // 计算每个轴的增量比例 (x方向固定步长避免近距离处畸变; 0.5是瞎设的防止过于稀疏导致错误)
            float dx = 0.5 * grid_map.raycast_step_;
            float dy = (y / x) * dx;
            float dz = (z / x) * dx;

            // 递增射线方向上的每个轴
            int scale = 0;
            float depth = 0.0f;

            while (1)
            {
                scale += 1;

                float point_x = scale * dx;
                float point_y = scale * dy;
                float point_z = scale * dz;

                float3 point_c = make_float3(point_x, point_y, point_z);
                float3 point_w = T_wc * point_c;

                Vector3f point(point_w.x, point_w.y, point_w.z);

                int occupied = grid_map.mapQuery(point);

                if (occupied == 1)
                {
                    // depth = point_x;  // 直接这样赋值会有一点误差
                    // 栅格化避免平面变曲面 (有些冗余，但在机体系栅格化会有类似摩尔纹的东西)
                    Vector3i occ_vox_w = grid_map.Pos2Vox(point);
                    Vector3f occ_point_w = grid_map.Vox2Pos(occ_vox_w);
                    float3 occ_point_w_ = make_float3(occ_point_w.x, occ_point_w.y, occ_point_w.z);
                    float3 occ_point_c_ = T_wc.inv() * occ_point_w_;
                    depth = occ_point_c_.x;
                    break;
                }

                if (point_x >= camera_param.max_depth_dist){
                    depth = camera_param.max_depth_dist;
                    break;
                }
            }

            // 将深度值存储到输出数组中
            if (camera_param.normalize_depth)
                depth = depth / camera_param.max_depth_dist;
            depth_values[v * camera_param.image_width + u] = depth;
        }
    }

    void renderDepthImage(GridMap* grid_map, CameraParams* camera_param, cudaMat::SE3<float>& T_wc, cv::Mat& depth_image)
    {   
        float* depth_values;
        size_t num_elements = camera_param->image_width * camera_param->image_height;
        cudaMallocManaged(&depth_values, num_elements * sizeof(float));

        // 在GPU上启动核函数
        cameraRaycastKernel<<<camera_param->image_height, camera_param->image_width>>>(depth_values, *grid_map, *camera_param, T_wc);
        
        cudaDeviceSynchronize();

        depth_image.create(camera_param->image_height, camera_param->image_width, CV_32FC1);

        cudaMemcpy(depth_image.data, depth_values, num_elements * sizeof(float), cudaMemcpyDeviceToHost);
        
        cudaFree(depth_values);
        return;
    }

    __global__ void lidarRaycastKernel(Vector3f* point_values, GridMap grid_map, LidarParams lidar_param, cudaMat::SE3<float> T_wc)
    {
        int h = threadIdx.x;
        int v = blockIdx.x;

        // printf("u: %d, v: %d \n", u, v);
        if (h < lidar_param.horizontal_num && v < lidar_param.vertical_lines)
        {   
            float vertical_resolution = (lidar_param.vertical_angle_end - lidar_param.vertical_angle_start) / (lidar_param.vertical_lines - 1);
            float vertical_angle = lidar_param.vertical_angle_start + v * vertical_resolution;
            float sin_vert = std::sin(vertical_angle * M_PI / 180.0);
            float cos_vert = std::cos(vertical_angle * M_PI / 180.0);
            float horizontal_angle = h * lidar_param.horizontal_resolution;
            float sin_horz = std::sin(horizontal_angle * M_PI / 180.0);
            float cos_horz = std::cos(horizontal_angle * M_PI / 180.0);
            // 计算射线方向
            Vector3f ray_direction(cos_vert * cos_horz, cos_vert * sin_horz, sin_vert);

            // 计算每个轴的增量比例 (0.5倍避免薄壁体素被跳过)
            float dx = ray_direction.x * 0.5f * grid_map.raycast_step_;
            float dy = ray_direction.y * 0.5f * grid_map.raycast_step_;
            float dz = ray_direction.z * 0.5f * grid_map.raycast_step_;

            // 递增射线方向上的每个轴
            int scale = 0;
            Vector3f point_value(0, 0, 0);

            while (1)
            {
                scale += 1;

                float point_x = scale * dx;
                float point_y = scale * dy;
                float point_z = scale * dz;

                float3 point_c = make_float3(point_x, point_y, point_z);
                float3 point_w = T_wc * point_c;

                Vector3f point(point_w.x, point_w.y, point_w.z);

                int occupied = grid_map.mapQuery(point);

                float ray_length = sqrtf(point_x * point_x + point_y * point_y + point_z * point_z);

                if (occupied == 1)
                {
                    Vector3i vox_w = grid_map.Pos2Vox(point);           // 在 world 系量化，避免平面变曲面
                    Vector3f occ_point_w = grid_map.Vox2Pos(vox_w);
                    float3 occ_w = make_float3(occ_point_w.x, occ_point_w.y, occ_point_w.z);
                    float3 occ_c = T_wc.inv() * occ_w;                  // 变换回 body 系
                    point_value = Vector3f(occ_c.x, occ_c.y, occ_c.z);
                    break;
                }

                if (ray_length > lidar_param.max_lidar_dist){
                    break;
                }
            }

            // 将点云值存储到输出数组中，(0, 0, 0)为无效值
            point_values[v * lidar_param.horizontal_num + h] = point_value;
        }
    }

    // 对 range image 每个像素直接射出球坐标系射线，用 3D DDA 精确穿越体素
    // 行列约定: row=0→仰角最大(elev_end), row=H-1→仰角最小(elev_start)
    //           col=0→方位角-π(正后), col=W/2→方位角0(正前)
    // 命中时 range = 射线进入该体素的 t 值（精确面交点），不做体素中心量化
    __global__ void rangeImageRaycastKernel(float* range_values, GridMap grid_map, LidarParams lidar_param, cudaMat::SE3<float> T_wb)
    {
        int col = threadIdx.x;
        int row = blockIdx.x;
        if (col >= lidar_param.horizontal_num || row >= lidar_param.vertical_lines) return;

        const float INF = 1e30f;
        const float PI  = 3.14159265358979323846f;

        float H = (float)lidar_param.vertical_lines;
        float W = (float)lidar_param.horizontal_num;
        float elev_max = lidar_param.vertical_angle_end   * PI / 180.0f;
        float elev_min = lidar_param.vertical_angle_start * PI / 180.0f;

        float elev = elev_max - (row / (H - 1.0f)) * (elev_max - elev_min);
        float azim = ((float)col / W) * 2.0f * PI - PI;

        float cos_e = cosf(elev), sin_e = sinf(elev);
        float cos_a = cosf(azim), sin_a = sinf(azim);
        float3 dir_b = make_float3(cos_e * cos_a, cos_e * sin_a, sin_e);  // FLU 机体系单位向量

        // 射线在世界系的原点和方向
        float3 O = T_wb.getTranslation();
        float3 D = T_wb.rotate(dir_b);  // 旋转（不加平移）

        float res = grid_map.getResolution();
        float ox  = grid_map.getOriginX();
        float oy  = grid_map.getOriginY();
        float oz  = grid_map.getOriginZ();

        // DDA 步进方向
        int stepX = (D.x >= 0.0f) ? 1 : -1;
        int stepY = (D.y >= 0.0f) ? 1 : -1;
        int stepZ = (D.z >= 0.0f) ? 1 : -1;

        // 射线原点所在体素
        int vox_x = (int)floorf((O.x - ox) / res);
        int vox_y = (int)floorf((O.y - oy) / res);
        int vox_z = (int)floorf((O.z - oz) / res);

        // 到各轴第一个体素壁的世界坐标
        float bx = ox + (float)(vox_x + (D.x >= 0.0f ? 1 : 0)) * res;
        float by = oy + (float)(vox_y + (D.y >= 0.0f ? 1 : 0)) * res;
        float bz = oz + (float)(vox_z + (D.z >= 0.0f ? 1 : 0)) * res;

        // 到各轴下一个体素壁的 t 值（D 为单位向量，t = 距离）
        float tMaxX  = (fabsf(D.x) > 1e-9f) ? (bx - O.x) / D.x : INF;
        float tMaxY  = (fabsf(D.y) > 1e-9f) ? (by - O.y) / D.y : INF;
        float tMaxZ  = (fabsf(D.z) > 1e-9f) ? (bz - O.z) / D.z : INF;
        // 穿越一个体素在各轴的 Δt
        float tDeltaX = (fabsf(D.x) > 1e-9f) ? res / fabsf(D.x) : INF;
        float tDeltaY = (fabsf(D.y) > 1e-9f) ? res / fabsf(D.y) : INF;
        float tDeltaZ = (fabsf(D.z) > 1e-9f) ? res / fabsf(D.z) : INF;

        float range_val = lidar_param.max_lidar_dist;
        float t = 0.0f;  // 当前体素入口的 t

        while (t < lidar_param.max_lidar_dist)
        {
            // 当前体素出口 t = 三轴中最小的 tMax
            float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
            // 在体素中点采样，确保点落在当前体素内部
            float t_mid = (t + t_exit) * 0.5f;
            float3 p = make_float3(O.x + t_mid * D.x, O.y + t_mid * D.y, O.z + t_mid * D.z);

            if (grid_map.mapQuery(Vector3f(p.x, p.y, p.z)))
            {
                // t 精确等于射线进入此体素的面交点距离
                range_val = t;
                break;
            }

            // 沿 tMax 最小的轴前进一格
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                t = tMaxX; tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                t = tMaxY; tMaxY += tDeltaY;
            } else {
                t = tMaxZ; tMaxZ += tDeltaZ;
            }
        }

        range_values[row * lidar_param.horizontal_num + col] = range_val;
    }

    void renderRangeImage(GridMap* grid_map, LidarParams* lidar_param, cudaMat::SE3<float>& T_wb, cv::Mat& range_image)
    {
        float* range_values;
        size_t num_elements = lidar_param->vertical_lines * lidar_param->horizontal_num;
        cudaMallocManaged(&range_values, num_elements * sizeof(float));

        rangeImageRaycastKernel<<<lidar_param->vertical_lines, lidar_param->horizontal_num>>>(
            range_values, *grid_map, *lidar_param, T_wb);

        cudaDeviceSynchronize();

        range_image.create(lidar_param->vertical_lines, lidar_param->horizontal_num, CV_32FC1);
        cudaMemcpy(range_image.data, range_values, num_elements * sizeof(float), cudaMemcpyDeviceToHost);

        cudaFree(range_values);
    }

    void renderLidarPointcloud(GridMap *grid_map, LidarParams *lidar_param, cudaMat::SE3<float>& T_wc, pcl::PointCloud<pcl::PointXYZ>& lidar_points){
        Vector3f* point_values;
        size_t num_elements = lidar_param->vertical_lines * lidar_param->horizontal_num;
        cudaMallocManaged(&point_values, num_elements * sizeof(Vector3f));

        // 在GPU上启动核函数
        lidarRaycastKernel<<<lidar_param->vertical_lines, lidar_param->horizontal_num>>>(point_values, *grid_map, *lidar_param, T_wc);
        
        cudaDeviceSynchronize();

        std::vector<Vector3f> cpu_points(num_elements);
        cudaMemcpy(cpu_points.data(), point_values, num_elements * sizeof(Vector3f), cudaMemcpyDeviceToHost);
        
        lidar_points.points.clear();
        lidar_points.points.reserve(num_elements);
        
        for (const auto& point : cpu_points) {
            if (point.x != 0 || point.y != 0 || point.z != 0) {
                lidar_points.points.emplace_back(point.x, point.y, point.z);
            }
        }
        cudaFree(point_values);
        return;
    }


    
}