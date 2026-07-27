#pragma once

#include <thread>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/String.h>    // user-input
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>


#include <common/types.h>
#include <common/camera_fwd.h>
#include <common/transformation.h>

namespace FAR {

// forward declarations
class FrameHandlerBase;
class Visualizer;
class ImuHandler;
// class BackendInterface;
// class CeresBackendInterface;
// class CeresBackendPublisher;

enum class PipelineType {
  kMono,
  kStereo,
  kArray
};

/// FARInterface 
class FARInterface
{
public:

  // ROS subscription and publishing.
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  PipelineType pipeline_type_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  std::unique_ptr<std::thread> imu_thread_;
  std::unique_ptr<std::thread> image_thread_;
  std::unique_ptr<std::thread> estimator_thread_;
  std::mutex img_mutex_;
  std::queue<sensor_msgs::ImageConstPtr> img0_buf_, img1_buf_;
  bool getImage(double &time, cv::Mat &img0, cv::Mat &img1);

  ros::Subscriber sub_imu;
  ros::Subscriber sub_img0, sub_img1;
  void subImage0CB(const sensor_msgs::ImageConstPtr &img_msg);
  void subImage1CB(const sensor_msgs::ImageConstPtr &img_msg);

  // FARInterface modules.
  std::shared_ptr<FrameHandlerBase> svo_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<ImuHandler> imu_handler_;
  // std::shared_ptr<BackendInterface> backend_interface_;
  // std::shared_ptr<CeresBackendInterface> ceres_backend_interface_;
  // std::shared_ptr<CeresBackendPublisher> ceres_backend_publisher_;

  CameraBundlePtr ncam_;
  Eigen::Vector3d position_init_offset_;

  // Parameters
  bool set_initial_attitude_from_gravity_ = true;

  // System state.
  bool quit_ = false;
  bool idle_ = false;
  bool automatic_reinitialization_ = false;

  FARInterface(const PipelineType& pipeline_type,
          const ros::NodeHandle& nh,
          const ros::NodeHandle& private_nh);

  virtual ~FARInterface();

  // Processing
  void processImageBundle(
      const std::vector<cv::Mat>& images,
      int64_t timestamp_nanoseconds);

  bool setImuPrior(const int64_t timestamp_nanoseconds);

  void publishResults(
      const std::vector<cv::Mat>& images,
      const int64_t timestamp_nanoseconds);

  // Subscription and callbacks
  void monoCallback(const sensor_msgs::ImageConstPtr& msg);
  void stereoCallback(
      const sensor_msgs::ImageConstPtr& msg0,
      const sensor_msgs::ImageConstPtr& msg1);
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void inputKeyCallback(const std_msgs::StringConstPtr& key_input);


  // These functions are called before and after monoCallback or stereoCallback.
  // a derived class can implement some additional logic here.
  virtual void imageCallbackPreprocessing(int64_t timestamp_nanoseconds) {}
  virtual void imageCallbackPostprocessing() {}

  void subscribeImu();
  void subscribeImage();
  void subscribeRemoteKey();

  void imuLoop();
  void monoLoop();
  void stereoLoop();
  void estimatorLoop();
};

} // namespace svo
