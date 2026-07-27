#ifndef ROLLING_GRID_MAP_MANAGER_HPP
#define ROLLING_GRID_MAP_MANAGER_HPP

#include "grid_map.hpp"
#include <array>
#include <memory>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace std
{
    template<> struct hash<Eigen::Vector3i>
    {
        size_t operator()(const Eigen::Vector3i& v) const
        {
            return ((hash<int>()(v.x()) ^ (hash<int>()(v.y()) << 1)) >> 1) ^ (hash<int>()(v.z()) << 1);
        }
    };
}

class RollingGridMapManager
{
public:
    RollingGridMapManager(const Eigen::Vector3d& mapSize_meters, double resolution)
        : mapSize_meters_(mapSize_meters), resolution_(resolution)
    {
        mapSize_grid_ = (mapSize_meters / resolution).cast<int>() + Eigen::Vector3i{1, 1, 1};

        for (auto& map : maps_)
        {
            map = std::make_unique<GridMap>(mapSize_grid_, resolution);
        }
    }

    void updatePosition(const Eigen::Vector3d& position)
    {
        if(have_odom_)
        {
            Eigen::Vector3i currentOrigin_id = last_center_id_;
            Eigen::Vector3d currentOrigin_id_double = (position.array() / mapSize_meters_.array());
            bool update_rolling_map = false;
            for( int i = 0; i < 3; i++ )
            {
                if( currentOrigin_id_double[i] - last_center_id_[i] > 1.2 )
                {
                    xyz_crossed_flag_[i] = 1;
                    currentOrigin_id[i]++;
                    update_rolling_map = true;
                }
                else if( currentOrigin_id_double[i] - last_center_id_[i] < -0.2 )
                {
                    xyz_crossed_flag_[i] = -1;
                    currentOrigin_id[i]--;
                    update_rolling_map = true;
                }
                else
                    xyz_crossed_flag_[i] = 0;
            }
            if (update_rolling_map)
            {
                shiftMaps(currentOrigin_id);
                last_center_id_ = currentOrigin_id;
            }
        }
        else
        {
            // init the MapOrigins
            int x = static_cast<int>(std::floor((position.x()) / mapSize_meters_.x()));
            int y = static_cast<int>(std::floor((position.y()) / mapSize_meters_.y()));
            int z = static_cast<int>(std::floor((position.z()) / mapSize_meters_.z()));

            Eigen::Vector3i currentOrigin_id{x, y, z};
            last_center_id_ = currentOrigin_id;
            initMapOrigins(currentOrigin_id);
            have_odom_ = true;
        }
    }

    void addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_new)
    {
        cloud_new->points.clear();
        if(have_odom_)
        {
            for(int i = 0; i < cloud->points.size(); i++)
            {

                //kdkd add limit
                const double height_limit  = 2.2;
                const double low_limit = 0.2;
                if(cloud->points[i].z < height_limit && cloud->points[i].z > low_limit)
                {
                    Eigen::Vector3d tp{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
                    int mapIndex = getMapIndex(tp);
                    if (mapIndex >= 0)
                    {
                        Eigen::Vector3d localPosition = tp - mapOrigins_[mapIndex];
                        if(maps_[mapIndex]->setOccupied(localPosition))
                        {
                            tp = localPosition + mapOrigins_[mapIndex];
                            pcl::PointXYZ tp_pcl;
                            tp_pcl.x = tp[0];
                            tp_pcl.y = tp[1];
                            tp_pcl.z = tp[2];
                            cloud_new->points.push_back(tp_pcl);
                        }
                    }
                    
                }
            }
            
            // debug_acc_pc_num_ += cloud_new->points.size();
            // int sum = history_sum_point_num_;
            // for (int i = 0; i < 27; ++i)
            // {
            //     sum += maps_[i]->getPointNum();
            // }
            // std::cout << "acc pc num： " << debug_acc_pc_num_ << std::endl;
            // std::cout << "map pc num： " << sum << std::endl;
            // std::cout << std::endl;
        }
    }

    bool isOccupied(const Eigen::Vector3d& position) const
    {
        int mapIndex = getMapIndex(position);
        if (mapIndex >= 0)
        {
            Eigen::Vector3d localPosition = position - mapOrigins_[mapIndex];
            return maps_[mapIndex]->isOccupied(localPosition);
        }
        return false;
    }

    void debugGetPointCloudFromMaps(pcl::PointCloud<pcl::PointXYZI>::Ptr &map_cloud)
    {
        map_cloud->points.clear();
        for (int i = 0; i < 27; ++i)
        {
            maps_[i]->debugGetPointCloudFromMap(map_cloud, mapOrigins_[i], i);
        }
    }

    void saveCurrentMaps()
    {
        for(int i = 0; i < 27; i++)
            maps_[i]->saveToFile(); 
    }

    std::array<Eigen::Vector3d, 27> getMapOrigins() const { return mapOrigins_; }
    Eigen::Vector3d getMapSizeMeters() const { return mapSize_meters_; }
    double getResolution() const { return resolution_; }
    bool getHaveOdom() const { return have_odom_; }

private:
    Eigen::Vector3d mapSize_meters_;
    Eigen::Vector3i mapSize_grid_;
    double resolution_;
    std::vector<Eigen::Vector3i> newMap_index_vec_;
    std::vector<Eigen::Vector3i> deleteMap_index_vec_;
    Eigen::Vector3i last_center_id_;
    Eigen::Vector3i xyz_crossed_flag_{0, 0, 0};
    std::array<std::unique_ptr<GridMap>, 27> maps_;  // 3x3x3 grid of maps
    std::array<Eigen::Vector3d, 27> mapOrigins_;
    Eigen::Vector3i center_ = Eigen::Vector3i::Zero();
    bool have_odom_ = false;
    std::unordered_map<Eigen::Vector3i, bool> saved_map_index_;
    int history_sum_point_num_ = 0;
    int debug_acc_pc_num_ = 0;

    // void shiftMaps(std::vector<Eigen::Vector3i> &newMap_index_vec, std::vector<Eigen::Vector3i> &deleteMap_index_vec, const Eigen::Vector3i& origin_id)
    void shiftMaps(const Eigen::Vector3i& origin_id)
    {
        std::array<std::unique_ptr<GridMap>, 27> newMaps;
        // std::array<bool, 27> removeOldMaps;
        // std::fill(removeOldMaps.begin(), removeOldMaps.end(), true);
        // std::vector<std::unique_ptr<GridMap>> deleteMaps;

        std::unordered_map<Eigen::Vector3i, int> now_index_vec;
        std::unordered_map<Eigen::Vector3i, int> old_index_vec;
        for (int i = 0; i < 27; ++i)
        {
            Eigen::Vector3i nowIndex(i % 3 - 1, (i / 3) % 3 - 1, i / 9 - 1);
            Eigen::Vector3i oldIndex = nowIndex - xyz_crossed_flag_;
            now_index_vec[nowIndex] = -(i+1);
            old_index_vec[oldIndex] = -(i+1);
        }

        // step.1: 检查新老地图重复的部分，进行交接
        for (auto& nowIndex : now_index_vec) 
        {
            int i = abs(nowIndex.second) -1;

            auto oldIndex_ptr = old_index_vec.find(nowIndex.first); 
            if (oldIndex_ptr != old_index_vec.end())
            {
                nowIndex.second = -nowIndex.second;
                oldIndex_ptr->second = -oldIndex_ptr->second;

                int oldFlatIndex = (oldIndex_ptr->second - 1);
                newMaps[i] = std::move(maps_[oldFlatIndex]);
            }
            else
            {
                newMaps[i] = std::make_unique<GridMap>(mapSize_grid_, resolution_);
            }
            mapOrigins_[i] = (origin_id+nowIndex.first).cast<double>().cwiseProduct(mapSize_meters_);
            newMaps[i]->setMapOriginInt(origin_id+nowIndex.first);
        }

        // step.2: 检查新老地图没有重合的部分
        std::vector<int> newMaps_need_load;
        std::vector<int> oldMaps_need_save;
        for (auto& nowIndex : now_index_vec) 
        {
            if(nowIndex.second < 0)
                newMaps_need_load.push_back(-nowIndex.second-1);
        }
        for (auto& oldIndex : old_index_vec) 
        {
            if(oldIndex.second < 0)
                oldMaps_need_save.push_back(-oldIndex.second-1);
        }

        // step.3: 把没有重复的部分，老地图进行save，新地图看情况进行load
        // 把掉出滑窗的地图进行save
        #ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
        #endif
        for(int kk = 0; kk < oldMaps_need_save.size(); kk++)
        {
            int i = oldMaps_need_save[kk];
            history_sum_point_num_ += maps_[i]->getPointNum();
            maps_[i]->saveToFile();
            Eigen::Vector3i mapOriginInt = maps_[i]->getMapOriginInt();
            saved_map_index_[mapOriginInt] = true;
        }
        // 把加入滑窗的地图进行检查，如果是之前save过的就load回来
        #ifdef MP_EN
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
        #endif
        for(int kk = 0; kk < newMaps_need_load.size(); kk++)
        {
            int i = newMaps_need_load[kk];
            Eigen::Vector3i mapOriginInt = newMaps[i]->getMapOriginInt();
            auto it = saved_map_index_.find(mapOriginInt);
            if (it != saved_map_index_.end()) 
            {
                newMaps[i]->loadFromFile();
                history_sum_point_num_ -= newMaps[i]->getPointNum();
                saved_map_index_.erase(it);
            }
        }

        maps_ = std::move(newMaps);
    }

    void initMapOrigins(const Eigen::Vector3i& origin_id)
    {
        for (int i = 0; i < 27; ++i)
        {
            Eigen::Vector3i offset(i % 3 - 1, (i / 3) % 3 - 1, i / 9 - 1);
            mapOrigins_[i] = (origin_id+offset).cast<double>().cwiseProduct(mapSize_meters_);
            maps_[i]->setMapOriginInt(origin_id+offset);
        }
    }

    int getMapIndex(const Eigen::Vector3d& position) const
    {
        int x = static_cast<int>(std::floor((position.x() - mapOrigins_[13].x()) / mapSize_meters_.x())) + 1;
        int y = static_cast<int>(std::floor((position.y() - mapOrigins_[13].y()) / mapSize_meters_.y())) + 1;
        int z = static_cast<int>(std::floor((position.z() - mapOrigins_[13].z()) / mapSize_meters_.z())) + 1;

        Eigen::Vector3i index{x, y, z};
        if ((index.array() >= 0).all() && (index.array() < 3).all())
        {
            return index.z() * 9 + index.y() * 3 + index.x();
        }
        return -1;
    }
};

#endif // ROLLING_GRID_MAP_MANAGER_HPP