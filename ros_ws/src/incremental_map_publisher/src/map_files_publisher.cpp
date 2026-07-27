#include "rolling_map/rolling_map_node.hpp"
#include "rolling_map/file_utils.hpp"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "map_files_publisher_node");
    ros::NodeHandle nh;

    int maxOnePCSize;
    int pc_pub_freq;
    Eigen::Vector3d mapSize_meters;
    Eigen::Vector3i mapSize_grid;
    double resolution;

    nh.param<int>("/max_one_pc_size", maxOnePCSize, 100000);
    nh.param<int>("/pc_pub_freq", pc_pub_freq, 10);
    nh.param<double>("/map_size_x_meters", mapSize_meters.x(), 50);
    nh.param<double>("/map_size_y_meters", mapSize_meters.y(), 50);
    nh.param<double>("/map_size_z_meters", mapSize_meters.z(), 50);
    nh.param<double>("/resolution", resolution, 0.1);
    mapSize_grid = (mapSize_meters / resolution).cast<int>() + Eigen::Vector3i{1, 1, 1};

    std::string dir_path(std::string(std::string(ROOT_DIR) + "map_files"));
    std::vector<std::string> map_files;
    if(!file_utils::checkAndLoadDirectoryName(dir_path, map_files))
    {
        std::cout << "\033[1;32m[map_files_publisher] load map files fail !!!" << "\033[0m" << std::endl;
        exit(0);
    }
        std::cout << map_files[0] << std::endl;

    std::vector<sensor_msgs::PointCloud2::Ptr> map_msgs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    int intensity = 0;
    int sum_pc_size = 0;
    for(int i = 0; i < map_files.size(); i++)
    {
        std::unique_ptr<GridMap> map = std::make_unique<GridMap>(mapSize_grid, resolution);
        Eigen::Vector3i origin_index = file_utils::stringToVector3i(map_files[i]);
        std::cout << map_files[i] << std::endl;
        std::cout << origin_index.transpose() << std::endl;

        map->setMapOriginInt(origin_index);
        map->loadFromFile();
        Eigen::Vector3d mapOrigin = (origin_index).cast<double>().cwiseProduct(mapSize_meters);
        map->debugGetPointCloudFromMap(cloud_ptr, mapOrigin, intensity);

        if(cloud_ptr->points.size() > maxOnePCSize)
        {
            sum_pc_size += cloud_ptr->points.size();
            sensor_msgs::PointCloud2::Ptr new_msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud_ptr, *new_msg);
            new_msg->header.frame_id = "world"; 
            map_msgs.push_back(new_msg);
            cloud_ptr->points.clear();
            intensity++;
        }
    }
    std::cout << "sum_pc_size: " << sum_pc_size << std::endl;

    ros::Publisher mapPub = nh.advertise<sensor_msgs::PointCloud2>("map_PC", 1);
    ros::Rate rate(pc_pub_freq);
    int i = 0;
    int map_count = map_msgs.size();
    while (ros::ok())
    {
        mapPub.publish(map_msgs[i]);
        i++;
        if(i == map_count)
            i = 0;
        rate.sleep();
    }

    return 0;
}