#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <unordered_map>
#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include "file_utils.hpp"
#include <pcl_conversions/pcl_conversions.h>

class GridMap
{
public:
    GridMap(const Eigen::Vector3i& size, double resolution)
        : size_(size), resolution_(resolution) {
            position_drift_ = Eigen::Vector3d::Constant(resolution_ * 0.5);
        }

    bool setOccupied(Eigen::Vector3d& position)
    {
        Eigen::Vector3i index = worldToGrid(position);
        if (isValidIndex(index))
        {
            uint32_t key = indexToKey(index);
            if(occupiedCells_.find(key) == occupiedCells_.end())
            {
                occupiedCells_[key] = true;
                // position = index.cast<double>() * resolution_;
                return true;
            }
        }
        return false;
    }

    bool isOccupied(const Eigen::Vector3d& position) const
    {
        Eigen::Vector3i index = worldToGrid(position);
        if (isValidIndex(index))
        {
            uint32_t key = indexToKey(index);
            return occupiedCells_.find(key) != occupiedCells_.end();
        }
        return false;
    }

    void clear()
    {
        occupiedCells_.clear();
    }

    void setMapOriginInt(Eigen::Vector3i origin_int)
    {
        mapOriginInt_ = origin_int;
    }

    void saveToFile()
    {
        std::string file_name = file_utils::vector3iToString(mapOriginInt_);
        std::string file_path(std::string(std::string(ROOT_DIR) + "map_files/") + file_name);

        file_utils::saveKeysToFile(occupiedCells_, file_path);
    }   

    void loadFromFile()
    {
        std::string file_name = file_utils::vector3iToString(mapOriginInt_);
        std::string file_path(std::string(std::string(ROOT_DIR) + "map_files/") + file_name);
        file_utils::loadKeysFromFile(occupiedCells_, file_path);
    }

    void debugGetPointCloudFromMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &map_cloud, Eigen::Vector3d mapOrigin, int intensity = 0)
    {
        for (const auto& pair : occupiedCells_) 
        {
            Eigen::Vector3i index = keyToIndex(pair.first);
            Eigen::Vector3d position = index.cast<double>() * resolution_ + mapOrigin;
            pcl::PointXYZI tp;
            tp.x = position[0];
            tp.y = position[1];
            tp.z = position[2];
            tp.intensity = intensity;
            map_cloud->points.push_back(tp);
        }
    }

    bool getKey(const Eigen::Vector3d& position, uint32_t &key) 
    {
        Eigen::Vector3i index = worldToGrid(position);
        if (isValidIndex(index))
        {
            key = indexToKey(index);
            return true;
        }
        return false;
    }

    Eigen::Vector3d getPoint(uint32_t key)
    {
        Eigen::Vector3i index = keyToIndex(key);
        return index.cast<double>() * resolution_;
    }

    double getResolution() const { return resolution_; }
    Eigen::Vector3i getSize() const { return size_; }
    Eigen::Vector3i getMapOriginInt() const { return mapOriginInt_; }
    int getPointNum() const { return occupiedCells_.size(); }

private:
    Eigen::Vector3i size_;
    double resolution_;
    Eigen::Vector3d position_drift_{0, 0, 0};
    std::unordered_map<uint32_t, bool> occupiedCells_;
    // 进入这个类的数据都是局部的，这个变量存在这里只是为了命名的时候方便索引
    Eigen::Vector3i mapOriginInt_;

    Eigen::Vector3i worldToGrid(const Eigen::Vector3d& position) const
    {
        // 这里加这个drift是为了让点栅格化之后对应一个格子的中心位置，否则的话会所有点都有一个往origin偏移的趋势（因为取整）
        return ((position + position_drift_) / resolution_).cast<int>();
    }

    bool isValidIndex(const Eigen::Vector3i& index) const
    {
        return (index.array() >= 0).all() && (index.array() < size_.array()).all();
    }

    uint32_t indexToKey(const Eigen::Vector3i& index) const
    {
        return static_cast<uint32_t>(index.z()) * size_.x() * size_.y() + 
               static_cast<uint32_t>(index.y()) * size_.x() + 
               static_cast<uint32_t>(index.x());
    }

    Eigen::Vector3i keyToIndex(uint32_t key) const 
    {
        int x = key % size_.x();
        int y = (key / size_.x()) % size_.y();
        int z = key / (size_.x() * size_.y());
        
        return Eigen::Vector3i(x, y, z);
    }
};

#endif // GRID_MAP_HPP