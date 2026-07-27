#ifndef ROLLING_MAP_NODE_HPP
#define ROLLING_MAP_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <Eigen/Eigen>
#include <pcl_conversions/pcl_conversions.h>
#include "rolling_grid_map_manager.hpp"
#include "file_utils.hpp"
#include "incremental_map_publisher/grid_PC.h"
#include "incremental_map_publisher/grid_PC_vec.h"
#include "incremental_map_publisher/rc_state.h"
#include <deque>
#include <mutex>

using std::string;

class RollingMapNode
{
public:
    RollingMapNode() : nh_("~")
    {
        Eigen::Vector3d mapSize_meters;
        double resolution;
        string odom_topic, pc_topic, communicationState_topic;
        double visMapBoundFreq, gridPCVecPubFreq;
        double bandwidth;


        nh_.param<double>("/map_size_x_meters", mapSize_meters.x(), 50);
        nh_.param<double>("/map_size_y_meters", mapSize_meters.y(), 50);
        nh_.param<double>("/map_size_z_meters", mapSize_meters.z(), 50);
        nh_.param<double>("/resolution", resolution, 0.1);
        nh_.param<string>("/communication_state_topic",communicationState_topic,"/wyj");
        nh_.param<string>("/point_cloud_topic",pc_topic,"/cloud_registered");
        nh_.param<string>("/odom_topic",odom_topic, "/Odometry");

        nh_.param<double>("/visualization_map_boundaries_freq", visMapBoundFreq, 1.0); 
        nh_.param<double>("/visualization_map_boundaries_color_r", visMapBoundColor_[0], 1.0);
        nh_.param<double>("/visualization_map_boundaries_color_g", visMapBoundColor_[1], 1.0);
        nh_.param<double>("/visualization_map_boundaries_color_b", visMapBoundColor_[2], 1.0);
        nh_.param<double>("/visualization_map_boundaries_color_a", visMapBoundColor_[3], 1.0);
        nh_.param<double>("/visualization_map_boundaries_line_size", visMapBoundLineSize_, 1.0);

        nh_.param<bool>("/debug/visualization_map_points_en", visMapPoints_en_, false);
        nh_.param<bool>("/debug/visualization_communication_points_en", visCommunicationPoints_en_, false);

        nh_.param<double>("/communication/grid_PC_vec_pub_freq", gridPCVecPubFreq, 10.0); 
        nh_.param<double>("/communication/bandwidth_kB_per_second", bandwidth, 50); 
        nh_.param<bool>("/communication/maximize_bandwidth_usage", maximizeBandwidthUsage_, true);

        //kdkd
        nh_.param<double>("/limit_z_height", limit_z_height, 2.5); 
        nh_.param<double>("/limit_z_low", limit_z_low, 0.5); 


        max_4Byte_num_ = int(((bandwidth * 1000) / 4.0) / gridPCVecPubFreq);
        have_communication_ = false;

        std::string dir_path(std::string(std::string(ROOT_DIR) + "map_files"));
        if( !file_utils::checkAndClearDirectory(dir_path) )
        {
            std::cout << "\033[1;32m[incremental_map_publisher] clear old map files fail !!!" << "\033[0m" << std::endl;
            exit(0);
        }
        std::cout << "\033[1;32m[incremental_map_publisher] clear old map files" << "\033[0m" << std::endl;

        initGridMapForCommunication();

        accCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("new_PC", 1);
        visMapBoundPub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_rolling_map_boundaries", 1);
        debugMapCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug/map_cloud", 1);
        debugCommunicationCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug/communication_cloud", 1);
        gridPCPub_ = nh_.advertise<incremental_map_publisher::grid_PC>("communication/grid_PC", 1);
        gridPCVecPub_ = nh_.advertise<incremental_map_publisher::grid_PC_vec>("communication/grid_PC_vec", 1);

        mapManager_ = std::make_unique<RollingGridMapManager>(mapSize_meters, resolution);

        communicationStateSub_ = nh_.subscribe(communicationState_topic, 1, &RollingMapNode::communicationStateCallback, this, ros::TransportHints().tcpNoDelay());
        odomSub_ = nh_.subscribe(odom_topic, 50, &RollingMapNode::odomCallback, this);
        pointCloudSub_ = nh_.subscribe(pc_topic, 50, &RollingMapNode::pointCloudCallback, this);
        gridPCVecPubTimer_ = nh_.createTimer(ros::Duration(1.0 / gridPCVecPubFreq), 
                                       &RollingMapNode::gridPCVecPubTimerCallback, this);
        visMapBoundTimer_ = nh_.createTimer(ros::Duration(1.0 / visMapBoundFreq), 
                                       &RollingMapNode::visMapBoundTimerCallback, this);

        if(visCommunicationPoints_en_)
        {
            debugCommunicationGridPCSub_ = nh_.subscribe("communication/grid_PC", 50, &RollingMapNode::gridPCCallback, this);
            debugCommunicationGridPCVecSub_ = nh_.subscribe("communication/grid_PC_vec", 50, &RollingMapNode::gridPCVecCallback, this);
        }
    }

    void saveCurrentMaps()
    {
        mapManager_->saveCurrentMaps(); 
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odomSub_;
    ros::Subscriber pointCloudSub_;
    ros::Subscriber debugCommunicationGridPCSub_, debugCommunicationGridPCVecSub_, communicationStateSub_;
    std::unique_ptr<RollingGridMapManager> mapManager_;
    std::unique_ptr<GridMap> gridMap_;
    Eigen::Vector3d min_xyz_;
    Eigen::Vector3d max_xyz_;
    ros::Publisher visMapBoundPub_, accCloudPub_, debugMapCloudPub_, debugCommunicationCloudPub_, gridPCPub_, gridPCVecPub_;
    ros::Timer visMapBoundTimer_, gridPCVecPubTimer_;
    Eigen::Vector4d visMapBoundColor_;
    Eigen::Vector3d odom_p_;
    double visMapBoundLineSize_;
    bool visMapPoints_en_ = false;
    bool visCommunicationPoints_en_ = false;
    uint32_t msg_id_ = 0;
    int max_4Byte_num_ = 0;
    bool maximizeBandwidthUsage_ = true;
    std::deque<incremental_map_publisher::grid_PC> grid_PC_buffer_;
    bool have_communication_ = false;
    double rc_orgin_time_ = -1;
    double ros_orgin_time_;
    double limit_z_height; 
    double limit_z_low; 
    // std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> lidar_buffer_;
    // std::deque<Eigen::Vector3d> odom_buffer_;
    // std::deque<double> lidar_time_buffer_;
    // std::deque<double> odom_time_buffer_;
    // std::mutex mtx_odom_buffer_;
    // std::mutex mtx_lidar_buffer_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        Eigen::Vector3d odom_p{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
        odom_p_ = odom_p;
        // mtx_odom_buffer_.lock();
        // odom_buffer_.push_bakc(odom_p);
        // odom_time_buffer_.push_back(msg->header.stamp.toSec());
        // mtx_odom_buffer_.unlock();

        // !
        mapManager_->updatePosition(odom_p);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // mtx_lidar_buffer_.lock();
        // lidar_buffer_.push_bakc(cloud);
        // lidar_time_buffer_.push_back(msg->header.stamp.toSec());
        // mtx_lidar_buffer_.unlock();

        // !
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
        mapManager_->addPointCloud(cloud, cloud_new);

        griddingPointCloudsForCommunication(cloud_new);

        sensor_msgs::PointCloud2::Ptr new_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_new, *new_msg);
        new_msg->header.frame_id = "world"; 
        accCloudPub_.publish(*new_msg);
    }

    void initGridMapForCommunication()
    {
        // todo 可以考虑写到参数文件里
        // (60*60*60)*(25*25*25)/((2**32)*1.0) = 0.78580342233181
        // (80*80*80)*(20*20*20)/((2**32)*1.0) = 0.95367431640625
        min_xyz_ = Eigen::Vector3d{-40, -40, -40};
        max_xyz_ = Eigen::Vector3d{ 40,  40,  40};
        double resolution = 0.05;

        Eigen::Vector3d mapSize_meters = max_xyz_ - min_xyz_;
        Eigen::Vector3i mapSize_grid = (mapSize_meters / resolution).cast<int>() + Eigen::Vector3i{1, 1, 1};
        gridMap_ = std::make_unique<GridMap>(mapSize_grid, resolution);
    }

    void griddingPointCloudsForCommunication(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new)
    {
        if(cloud_new->points.size() == 0)
            return;

        Eigen::Vector3d origin = odom_p_ + min_xyz_;

        std::vector<uint32_t> key_vec;
        int outlier_count = 0;
        for(int i = 0; i < cloud_new->points.size(); i++)
        {


            //kdkd
            if(cloud_new->points[i].z > limit_z_low  && cloud_new->points[i].z < limit_z_height)
            {
                Eigen::Vector3d tp{cloud_new->points[i].x, cloud_new->points[i].y, cloud_new->points[i].z};
                tp = tp - origin;
                uint32_t key;
                if(gridMap_->getKey(tp, key))
                {
                    key_vec.push_back(key);
                }
                else
                    outlier_count++;
            }
        }
        // if(outlier_count != 0)
        //     std::cout << "\033[1;32m[incremental_map_publisher] outlier_count: " << outlier_count << "\033[0m" << std::endl;

        incremental_map_publisher::grid_PC gridPC_msg;
        gridPC_msg.origin_x = origin.x();
        gridPC_msg.origin_y = origin.y();
        gridPC_msg.origin_z = origin.z();
        gridPC_msg.id = msg_id_;
        // gridPC_msg.time_in_rc = ros::Time::now().toSec(); // 此时不一定连接，所以存到buffer里的是ros时间，处理buffer数据时肯定连接了，那个时候转成rc时间
        gridPC_msg.key_size = key_vec.size();
        gridPC_msg.key_vec = key_vec;
        msg_id_++;
        grid_PC_buffer_.push_back(gridPC_msg);
    }

    void gridPCCallback(const incremental_map_publisher::grid_PC::ConstPtr& msg)
    {
        Eigen::Vector3d origin;
        origin.x() = msg->origin_x;
        origin.y() = msg->origin_y;
        origin.z() = msg->origin_z;

        int key_vec_size = msg->key_size;
        pcl::PointCloud<pcl::PointXYZ>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i = 0; i < key_vec_size; i++)
        {
            Eigen::Vector3d tp = gridMap_->getPoint(msg->key_vec[i]) + origin;
            pcl::PointXYZ pcl_tp;
            pcl_tp.x = tp[0];
            pcl_tp.y = tp[1];
            pcl_tp.z = tp[2];
            debug_cloud->points.push_back(pcl_tp);
        }
        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*debug_cloud, *cloud_msg);
        cloud_msg->header.frame_id = "world"; 
        debugCommunicationCloudPub_.publish(*cloud_msg);
    }

    void gridPCVecCallback(const incremental_map_publisher::grid_PC_vec::ConstPtr& msg)
    {
        int msg_num = msg->grid_PC_vec.size();
        pcl::PointCloud<pcl::PointXYZ>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(int msg_id = 0; msg_id < msg_num; msg_id++)
        {
            Eigen::Vector3d origin;
            origin.x() = msg->grid_PC_vec[msg_id].origin_x;
            origin.y() = msg->grid_PC_vec[msg_id].origin_y;
            origin.z() = msg->grid_PC_vec[msg_id].origin_z;

            int key_vec_size = msg->grid_PC_vec[msg_id].key_size;
            for(int i = 0; i < key_vec_size; i++)
            {
                Eigen::Vector3d tp = gridMap_->getPoint(msg->grid_PC_vec[msg_id].key_vec[i]) + origin;
                pcl::PointXYZ pcl_tp;
                pcl_tp.x = tp[0];
                pcl_tp.y = tp[1];
                pcl_tp.z = tp[2];
                debug_cloud->points.push_back(pcl_tp);
            }
        }
        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*debug_cloud, *cloud_msg);
        cloud_msg->header.frame_id = "world"; 
        debugCommunicationCloudPub_.publish(*cloud_msg);
    }


    void communicationStateCallback(const incremental_map_publisher::rc_state::ConstPtr& msg)
    {
        have_communication_ = msg->connected; 
        if(rc_orgin_time_ < 0 && msg->connected)
        {
            rc_orgin_time_ = msg->cur_time;
            ros_orgin_time_ = ros::Time::now().toSec();
        }
    }

    void gridPCVecPubTimerCallback(const ros::TimerEvent&)
    {
        if(!have_communication_)
            return;

        std::vector<incremental_map_publisher::grid_PC> msg_vec;
        int vec_4Byte_num = 0;
        while(!grid_PC_buffer_.empty())
        {
            int current_msg_4Byte_num = grid_PC_buffer_.front().key_size + 5;
            if( vec_4Byte_num + current_msg_4Byte_num <= max_4Byte_num_ )
            {
                msg_vec.push_back(grid_PC_buffer_.front());
                vec_4Byte_num += current_msg_4Byte_num;
                grid_PC_buffer_.pop_front();
            }
            else
            {
                if(maximizeBandwidthUsage_)
                {
                    int remain_point_4Byte_num = max_4Byte_num_ - vec_4Byte_num - 5;
                    if( remain_point_4Byte_num > 0 )
                    {
                        incremental_map_publisher::grid_PC last_msg;
                        last_msg.origin_x = grid_PC_buffer_.front().origin_x;
                        last_msg.origin_y = grid_PC_buffer_.front().origin_y;
                        last_msg.origin_z = grid_PC_buffer_.front().origin_z;
                        last_msg.id = grid_PC_buffer_.front().id;
                        last_msg.key_size = remain_point_4Byte_num;
                        std::vector<uint32_t> &key_vec = grid_PC_buffer_.front().key_vec;
                        std::vector<uint32_t> front_new_key_vec;
                        std::vector<uint32_t> back_new_key_vec;
                        for(int i = 0; i < remain_point_4Byte_num; i++)
                        {
                            front_new_key_vec.push_back(key_vec[i]);
                        }
                        for(int i = remain_point_4Byte_num; i < grid_PC_buffer_.front().key_size; i++)
                        {
                            back_new_key_vec.push_back(key_vec[i]);
                        }
                        last_msg.key_vec = front_new_key_vec;
                        msg_vec.push_back(last_msg);

                        grid_PC_buffer_.front().key_size = back_new_key_vec.size();
                        grid_PC_buffer_.front().key_vec  = back_new_key_vec;
                    }
                    break;
                }
                else
                    break;
            }
        }

        if((!grid_PC_buffer_.empty()) && (msg_vec.size()==0))
        {
            if(maximizeBandwidthUsage_)
            {
                ROS_ERROR("[incremental_map_publisher] ERROR in func gridPCVecPubTimerCallback() !!!");
                ROS_ERROR("[incremental_map_publisher] maybe bandwidth_kB_per_second is too small !!!");
                exit(0);
            }
            else
            {
                ROS_ERROR("[incremental_map_publisher] The parameter bandwidth_kB_per_second is set so small that the number of points I have to send at a time exceeds the bandwidth, so I can't send them out. Consider setting the parameter maximize_bandwidth_usage to true");
                exit(0);
            }
        }
 
        incremental_map_publisher::grid_PC_vec new_msg;
        new_msg.grid_PC_vec = msg_vec;

        if( msg_vec.size() != 0 )
            gridPCVecPub_.publish(new_msg);
    }

    void visMapBoundTimerCallback(const ros::TimerEvent&)
    {
        if(mapManager_->getHaveOdom())
        {
            publishMapBoundVisualization();

            if( visMapPoints_en_ )
                publishMapPointsVisualization();
        }
    }

    void publishMapPointsVisualization()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        mapManager_->debugGetPointCloudFromMaps(map_cloud);

        sensor_msgs::PointCloud2::Ptr map_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*map_cloud, *map_msg);
        map_msg->header.frame_id = "world"; 
        debugMapCloudPub_.publish(*map_msg);
    }

    void publishMapBoundVisualization()
    {
        visualization_msgs::MarkerArray markerArray;
        int id = 0;

        std::array<Eigen::Vector3d, 27> mapOrigins = mapManager_->getMapOrigins();
        Eigen::Vector3d mapSize_meters = mapManager_->getMapSizeMeters();

        std::vector<Eigen::Vector3d> line_connect;
        line_connect.emplace_back(0,0,0);
        line_connect.emplace_back(1,0,0);
        line_connect.emplace_back(1,0,0);
        line_connect.emplace_back(1,1,0);
        line_connect.emplace_back(1,1,0);
        line_connect.emplace_back(0,1,0);
        line_connect.emplace_back(0,1,0);
        line_connect.emplace_back(0,0,0);
        line_connect.emplace_back(0,0,1);
        line_connect.emplace_back(1,0,1);
        line_connect.emplace_back(1,0,1);
        line_connect.emplace_back(1,1,1);
        line_connect.emplace_back(1,1,1);
        line_connect.emplace_back(0,1,1);
        line_connect.emplace_back(0,1,1);
        line_connect.emplace_back(0,0,1);
        line_connect.emplace_back(0,0,0);
        line_connect.emplace_back(0,0,1);
        line_connect.emplace_back(1,0,0);
        line_connect.emplace_back(1,0,1);
        line_connect.emplace_back(1,1,0);
        line_connect.emplace_back(1,1,1);
        line_connect.emplace_back(0,1,0);
        line_connect.emplace_back(0,1,1);

        for (int i = 0; i < 27; ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "rolling_map_boundaries";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = visMapBoundLineSize_;
            marker.color.r = visMapBoundColor_[0];
            marker.color.g = visMapBoundColor_[1];
            marker.color.b = visMapBoundColor_[2];
            marker.color.a = visMapBoundColor_[3];

            std::vector<geometry_msgs::Point> vertices;
            for( int ii = 0; ii < line_connect.size(); ii++ )
            {
                geometry_msgs::Point tp;
                tp.x = mapOrigins[i].x() + mapSize_meters.x() * line_connect[ii].x(),
                tp.y = mapOrigins[i].y() + mapSize_meters.y() * line_connect[ii].y(),
                tp.z = mapOrigins[i].z() + mapSize_meters.z() * line_connect[ii].z();
                marker.points.push_back(tp);
            }

            markerArray.markers.push_back(marker);
        }

        visMapBoundPub_.publish(markerArray);
    }
};

#endif // ROLLING_MAP_NODE_HPP