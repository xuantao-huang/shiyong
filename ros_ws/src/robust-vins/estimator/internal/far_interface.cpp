// Modification Note: 
// This file may have been modified by the authors of SchurVINS.
// (All authors of SchurVINS are with PICO department of ByteDance Corporation)
#include <estimator/far_interface.h>

#include <ros/callback_queue.h>

#include <estimator/far_factory.h>
#include <estimator/visualizer.h>
#include <common/frame.h>
#include <estimator/map.h>
#include <estimator/imu_handler.h>
#include <common/camera.h>
#include <common/conversions.h>
// #include <estimator/frame_handler_mono.h>
#include <estimator/frame_handler_stereo.h>
// #include <estimator/frame_handler_array.h>
#include <estimator/initialization.h>
#include <tracker/direct/depth_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <common/param_helper.h>
#include <vikit/timer.h>
// #include <svo_ros/ceres_backend_factory.h>
#include <opencv2/opencv.hpp>
#include <common/tic_toc.h>

namespace FAR {

FARInterface::FARInterface(
    const PipelineType& pipeline_type,
    const ros::NodeHandle& nh,
    const ros::NodeHandle& private_nh)
  : nh_(nh)
  , pnh_(private_nh)
  , pipeline_type_(pipeline_type)
  , set_initial_attitude_from_gravity_(
      vk::param<bool>(pnh_, "set_initial_attitude_from_gravity", true))
  , automatic_reinitialization_(
      vk::param<bool>(pnh_, "automatic_reinitialization", false))
{
  switch (pipeline_type)
  {
    // case PipelineType::kMono:
    //   svo_ = factory::makeMono(pnh_);
    //   break;
    case PipelineType::kStereo:
      svo_ = factory::makeStereo(pnh_);
      break;
    // case PipelineType::kArray:
    //   svo_ = factory::makeArray(pnh_);
    //   break;
    default:
      LOG(FATAL) << "Unknown pipeline";
      break;
  }
  ncam_ = svo_->getNCamera();

  visualizer_.reset(
        new Visualizer(svo_->options_.trace_dir, pnh_, ncam_->getNumCameras()));

  if(vk::param<bool>(pnh_, "use_imu", false))
  {
    imu_handler_ = factory::getImuHandler(pnh_);
    svo_->setImuHandler(imu_handler_);
    // svo_->imu_handler_ = imu_handler_;
  }

  const char *init_x = std::getenv("ipx");
  const char *init_y = std::getenv("ipy");
  const char *init_z = std::getenv("ipz");
  if (!init_x || !init_y || !init_z) {
    std::cout << "\033[0;1;34m[WARNING] COULD NOT FOUND INIT POSITION OFFSET, SET TO ZERO!\033[0m" << std::endl;
    position_init_offset_.setZero();
  } else {
    try {
      position_init_offset_.x() = std::stod(std::string(init_x));
      position_init_offset_.y() = std::stod(std::string(init_y));
      position_init_offset_.z() = std::stod(std::string(init_z));
    } catch (std::exception &e) {
      std::cout << "\033[0;1;34m""[ERROR] catch exception : " << e.what() << "\033[0m" << std::endl;
    }
    std::cout << "\033[0;1;34m""[INFO] SET INIT POSITION OFFSET " 
              << position_init_offset_.transpose() << "\033[0m" << std::endl;
  }

  estimator_thread_ = std::unique_ptr<std::thread>(
        new std::thread(&FARInterface::estimatorLoop, this));

  svo_->start();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1")
  {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();
  return img;
}

bool FARInterface::getImage(double &time, cv::Mat &image0, cv::Mat &image1)
{
  if (img0_buf_.empty()) 
  {
    return false;
  }
  std_msgs::Header header;
  img_mutex_.lock();
  if (!img0_buf_.empty() && !img1_buf_.empty())
  {
    double time0 = img0_buf_.front()->header.stamp.toSec();
    double time1 = img1_buf_.front()->header.stamp.toSec();
    // 0.003s sync tolerance
    if (time0 < time1 - 0.003)
    {
      img0_buf_.pop();
      std::cout << YELLOW << "[WARNING]: " << "THROW Image0" << TAIL << std::endl;
      img_mutex_.unlock();
      return false;
    }
    else if (time0 > time1 + 0.003)
    {
      img1_buf_.pop();
      std::cout << YELLOW << "[WARNING]: " << "THROW Image1" << TAIL << std::endl;
      img_mutex_.unlock();
      return false;
    }
    else
    {
      time = img0_buf_.front()->header.stamp.toNSec();
      header = img0_buf_.front()->header;
      image0 = getImageFromMsg(img0_buf_.front());
      img0_buf_.pop();
      image1 = getImageFromMsg(img1_buf_.front());
      img1_buf_.pop();
      if (!img0_buf_.empty() || !img1_buf_.empty()) 
      {
        std::cout << YELLOW << "[WARNING]: " << "Image Update Late, CURR Queue have " 
                  << img0_buf_.size() << ", " << img1_buf_.size() << " Image" << TAIL << std::endl;
      }
      img_mutex_.unlock();
      return true;
    }
  }
  img_mutex_.unlock();
  return false;
}

void FARInterface::estimatorLoop() 
{
  double last_img_timestamp = 0.;
  double curr_img_timestamp = 0.;
  bool first_image = true;
  double kImageFPS = vk::param<int>(pnh_, "image_fps", 15);
  double kMinUpdateCost = (1./kImageFPS)*1000.0;
  double kMinImageDt = (1./kImageFPS)*1000.0 + 5.;
  while (ros::ok())
  {
    cv::Mat img0, img1;
    double time;
    if (!getImage(time, img0, img1)) {
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
      continue;
    }
    LOG(INFO) << "------------------------------------";
    if (first_image) {
      first_image = false;
      last_img_timestamp = curr_img_timestamp = time;
    } else {
      curr_img_timestamp = time;
      double image_dt = (curr_img_timestamp - last_img_timestamp) * 1e-6;
      if (image_dt > kMinImageDt) {
        std::cout << YELLOW << "[WARNING]: " << "Unstable image frame rate, Image delay, Delta T for Image is " 
                  << image_dt << TAIL << "\n";
      } else if (image_dt < kMinImageDt/3.) {
        std::cout << YELLOW << "[WARNING]: " << "Unstable image frame rate, Image Too Fast, Delta T for Image is " 
                  << image_dt << TAIL << "\n";
      }
      last_img_timestamp = curr_img_timestamp;
      // std::cout << "IMG DT: " << image_dt << " IMG FPS: " << 1000./image_dt << std::endl;
    }
    // if (!setImuPrior(time))
    // {
    //   VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    //   return;
    // }

    assert(!img0.empty() && !img1.empty());
    TicToc estimator_t;
    processImageBundle({img0, img1}, time);
    double estimator_cost = estimator_t.toc();
    estimator_t.tic();
    publishResults({img0, img1}, time);
    double publish_cost = estimator_t.toc();

    if (estimator_cost+publish_cost > kMinUpdateCost) {
    // if (1) {
      std::cout << RED << "[WARNING]: " << "Updates take slightly longer and"
                << " may result in unpredictable behavior, UPDATE TIMES " 
                << estimator_cost << " + " << publish_cost << ", MIN COST is " 
                << kMinUpdateCost << TAIL << "\n";
      LOG(INFO) << "Updates take slightly longer and"
                << " may result in unpredictable behavior, UPDATE TIMES " 
                << estimator_cost << " + " << publish_cost << ", MIN COST is " 
                << kMinUpdateCost;
    }
    // std::cout << "PROCESS COST: " << estimator_cost+publish_cost << std::endl;
    std::chrono::milliseconds dura(1);
    std::this_thread::sleep_for(dura);
  }
}

FARInterface::~FARInterface()
{
  if (imu_thread_)
    imu_thread_->join();
  if (image_thread_)
    image_thread_->join();
  VLOG(1) << "Destructed SVO.";
}

void FARInterface::processImageBundle(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  // if (!svo_->isBackendValid())
  // {
  //   if (vk::param<bool>(pnh_, "use_ceres_backend", false, true))
  //   {
  //     ceres_backend_interface_ =
  //         ceres_backend_factory::makeBackend(pnh_, ncam_);
  //     if (imu_handler_)
  //     {
  //       svo_->setBundleAdjuster(ceres_backend_interface_);
  //       ceres_backend_interface_->setImu(imu_handler_);
  //       ceres_backend_interface_->makePublisher(pnh_, ceres_backend_publisher_);
  //     }
  //     else
  //     {
  //       SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
  //     }
  //   }
  // }
  svo_->addImageBundle(images, timestamp_nanoseconds);
}

void FARInterface::publishResults(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  CHECK_NOTNULL(svo_.get());
  CHECK_NOTNULL(visualizer_.get());

  visualizer_->img_caption_.clear();
  // if (svo_->isBackendValid())
  // {
  //   std::string static_str = ceres_backend_interface_->getStationaryStatusStr();
  //   visualizer_->img_caption_ = static_str;
  // }

  // visualizer_->publishSvoInfo(svo_.get(), timestamp_nanoseconds);
  switch (svo_->stage())
  {
    case Stage::kTracking: {
      Eigen::Matrix<double, 6, 6> covariance;
      covariance.setZero();
      visualizer_->publishImuPose(
            svo_->getLastFrames()->get_T_W_B(), covariance, timestamp_nanoseconds);
      visualizer_->publishCameraPoses(svo_->getLastFrames(), timestamp_nanoseconds);
      visualizer_->visualizeMarkers(
            svo_->getLastFrames(), svo_->closeKeyframes(), svo_->map());
      // visualizer_->exportToDense(svo_->getLastFrames());
      bool draw_boundary = false;
      // if (svo_->isBackendValid())
      // {
      //   draw_boundary = svo_->getBundleAdjuster()->isFixedToGlobalMap();
      // }
      visualizer_->publishImagesWithFeatures(
            svo_->getLastFrames(), timestamp_nanoseconds,
            draw_boundary);
#ifdef SVO_LOOP_CLOSING
      // detections
      if (svo_->lc_)
      {
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->cur_loop_check_viz_info_,
              std::string("loop_query"),
              Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.5);
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->loop_detect_viz_info_, std::string("loop_detection"),
              Eigen::Vector3f(1.0f, 0.0f, 0.0f), 1.0);
        if (svo_->isBackendValid())
        {
          visualizer_->publishLoopClosureInfo(
                svo_->lc_->loop_correction_viz_info_,
                std::string("loop_correction"),
                Eigen::Vector3f(0.0f, 1.0f, 0.0f), 3.0);
        }
        if (svo_->getLastFrames()->at(0)->isKeyframe())
        {
          bool pc_recalculated = visualizer_->publishPoseGraph(
                svo_->lc_->kf_list_,
                svo_->lc_->need_to_update_pose_graph_viz_,
                static_cast<size_t>(svo_->lc_->options_.ignored_past_frames));
          if(pc_recalculated)
          {
            svo_->lc_->need_to_update_pose_graph_viz_ = false;
          }
        }
      }
#endif
#ifdef SVO_GLOBAL_MAP
      if (svo_->global_map_)
      {
        visualizer_->visualizeGlobalMap(*(svo_->global_map_),
                                        std::string("global_vis"),
                                        Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                                        0.3);
        visualizer_->visualizeFixedLandmarks(svo_->getLastFrames()->at(0));
      }
#endif
      break;
    }
    case Stage::kInitializing: {
      visualizer_->publishBundleFeatureTracks(
            svo_->initializer_->frames_ref_, svo_->getLastFrames(),
            timestamp_nanoseconds);
      break;
    }
    case Stage::kPaused:
    case Stage::kRelocalization:
      visualizer_->publishImages(images, timestamp_nanoseconds);
      break;
    default:
      LOG(FATAL) << "Unknown stage";
      break;
  }

#ifdef SVO_USE_GTSAM_BACKEND
  if(svo_->stage() == Stage::kTracking && backend_interface_)
  {
    if(svo_->getLastFrames()->isKeyframe())
    {
      std::lock_guard<std::mutex> estimate_lock(backend_interface_->optimizer_->estimate_mut_);
      const gtsam::Values& state = backend_interface_->optimizer_->estimate_;
      ceres_backend_publisher_->visualizeFrames(state);
      if(backend_interface_->options_.add_imu_factors)
        ceres_backend_publisher_->visualizeVelocity(state);
      ceres_backend_publisher_->visualizePoints(state);
    }
  }
#endif
}

bool FARInterface::setImuPrior(const int64_t timestamp_nanoseconds)
{
  if(svo_->getBundleAdjuster())
  {
    //if we use backend, this will take care of setting priors
    if(!svo_->hasStarted())
    {
      //when starting up, make sure we already have IMU measurements
      if(imu_handler_->getMeasurementsCopy().size() < 10u)
      {
        return false;
      }
    }
    return true;
  }

  if(imu_handler_ && !svo_->hasStarted() && set_initial_attitude_from_gravity_)
  {
    // set initial orientation
    Quaternion R_imu_world;
    if(imu_handler_->getInitialAttitude(
         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
         R_imu_world))
    {
      VLOG(3) << "Set initial orientation from accelerometer measurements.";
      svo_->setRotationPrior(R_imu_world);
    }
    else
    {
      return false;
    }
  }
  else if(imu_handler_ && svo_->getLastFrames())
  {
    // set incremental rotation prior
    Quaternion R_lastimu_newimu;
    if(imu_handler_->getRelativeRotationPrior(
         svo_->getLastFrames()->getMinTimestampNanoseconds() *
         common::conversions::kNanoSecondsToSeconds,
         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
         false, R_lastimu_newimu))
    {
      VLOG(3) << "Set incremental rotation prior from IMU.";
      svo_->setRotationIncrementPrior(R_lastimu_newimu);
    }
  }
  return true;
}

void FARInterface::monoCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(idle_)
    return;

  cv::Mat image;
  try
  {
    image = cv_bridge::toCvCopy(msg)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  std::vector<cv::Mat> images;
  images.push_back(image.clone());

  if(!setImuPrior(msg->header.stamp.toNSec()))
  {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(msg->header.stamp.toNSec());

  processImageBundle(images, msg->header.stamp.toNSec());


  publishResults(images, msg->header.stamp.toNSec());

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_)
    svo_->start();

  imageCallbackPostprocessing();
}

void FARInterface::stereoCallback(
    const sensor_msgs::ImageConstPtr& msg0,
    const sensor_msgs::ImageConstPtr& msg1)
{
  if(idle_)
    return;

  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // {
  //   double clahe_value = 10.0;
  //   cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clahe_value, cv::Size(8, 8));
  //   clahe->apply(img0, img0);
  //   clahe->apply(img1, img1);
  // }

  if(!setImuPrior(msg0->header.stamp.toNSec()))
  {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(msg0->header.stamp.toNSec());

  processImageBundle({img0, img1}, msg0->header.stamp.toNSec());
  publishResults({img0, img1}, msg0->header.stamp.toNSec());

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_)
    svo_->start();

  imageCallbackPostprocessing();
}

void FARInterface::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  TicToc imu_cb_cost;
  constexpr double kImuRate = 50;
  constexpr double kImuDt = 1./kImuRate;
  static double last_imu_time = 0., curr_imu_time = 0.0;
  static bool first_imu = true;
  if (first_imu)
  {
    first_imu = false;
    last_imu_time = curr_imu_time = msg->header.stamp.toSec();
  } 
  else
  {
    curr_imu_time = msg->header.stamp.toSec();
    double imu_dt = (curr_imu_time-last_imu_time);
    if (imu_dt > kImuDt)
    {
      std::cout << YELLOW << "[WARNING]: " << "IMU Dt " << imu_dt*1000.
                << " ms, IMU FPS " << 1./imu_dt << ", MIN DT: " << kImuDt << TAIL << std::endl;
    }
    last_imu_time = curr_imu_time;
  }
  const Eigen::Vector3d omega_imu(
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d lin_acc_imu(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  ImuMeasurement m(msg->header.stamp.toSec(), omega_imu, lin_acc_imu);
  Eigen::Quaterniond mavros_q;
  mavros_q.w() = msg->orientation.w;
  mavros_q.x() = msg->orientation.x;
  mavros_q.y() = msg->orientation.y;
  mavros_q.z() = msg->orientation.z;

  if(imu_handler_)
    imu_handler_->addImuMeasurement(m);
  else
    SVO_ERROR_STREAM("SvoNode has no ImuHandler");
  if (svo_->getImuRatePoseInitSuccess()) 
  {
    FrameHandlerBase::ImuLatestState state;
    svo_->imuRatePropagation(msg->header.stamp.toSec(), lin_acc_imu, omega_imu, state);


    Eigen::Quaterniond ORB = state.latest_Q_;
    Eigen::Quaterniond FRB = mavros_q;
    Eigen::Quaterniond FRO = FRB * ORB.inverse();
    Eigen::Matrix3d ORF = FRO.inverse().toRotationMatrix();
    Eigen::Quaterniond WRF = Eigen::AngleAxisd(atan2(ORF(1, 0), ORF(0, 0)),Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(0,Vector3d::UnitY()) *   
                      Eigen::AngleAxisd(0,Vector3d::UnitX());
    Eigen::Quaterniond WRO = WRF * FRO;
    Eigen::Quaterniond WRB = WRO * ORB;

    visualizer_->publishImuOdometry(state.latest_P_ + position_init_offset_, WRB, 
                                    state.latest_V_, state.latest_time_, state.latest_Ba_, state.latest_Bg_);
  }
  if (imu_cb_cost.toc() > 5.)
  {
    std::cout << YELLOW << "[VINS:ERROR] IMU CB COST TIME " << imu_cb_cost.toc() << TAIL << std::endl;
  }
}


void FARInterface::subscribeImu()
{
  // imu_thread_ = std::unique_ptr<std::thread>(
  //       new std::thread(&FARInterface::imuLoop, this));
  // sleep(3);
  imuLoop();
}

void FARInterface::subscribeImage()
{
  // if(pipeline_type_ == PipelineType::kMono)
  //   image_thread_ = std::unique_ptr<std::thread>(
  //         new std::thread(&FARInterface::monoLoop, this));
  // else if(pipeline_type_ == PipelineType::kStereo)
    // image_thread_ = std::unique_ptr<std::thread>(
    //     new std::thread(&FARInterface::stereoLoop, this));
    stereoLoop();
}

void FARInterface::subscribeRemoteKey()
{
  // std::string remote_key_topic =
  //     vk::param<std::string>(pnh_, "remote_key_topic", "svo/remote_key");
  // sub_remote_key_ =
  //     nh_.subscribe(remote_key_topic, 5, &FAR::FARInterface::inputKeyCallback, this);
}

void FARInterface::imuLoop()
{
  SVO_INFO_STREAM("Started IMU loop.");
  // ros::NodeHandle nh;
  // ros::CallbackQueue queue;
  // nh.setCallbackQueue(&queue);
  std::string imu_topic = vk::param<std::string>(pnh_, "imu_topic", "imu");
  /*ros::Subscriber*/ sub_imu =
      pnh_.subscribe(imu_topic, 1000, &FAR::FARInterface::imuCallback, this, ros::TransportHints().tcpNoDelay());
  // while(ros::ok() && !quit_)
  // {
  //   queue.callAvailable(ros::WallDuration(0.1));
  // }
}

void FARInterface::monoLoop()
{
  /*
  SVO_INFO_STREAM("SvoNode: Started Image loop.");

  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  image_transport::ImageTransport it(nh);
  std::string image_topic =
      vk::param<std::string>(pnh_, "cam0_topic", "camera/image_raw");
  image_transport::Subscriber it_sub =
      it.subscribe(image_topic, 5, &FAR::FARInterface::monoCallback, this);

  while(ros::ok() && !quit_)
  {
    queue.callAvailable(ros::WallDuration(0.1));
  }
  */
}

void FARInterface::stereoLoop()
{
  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
  // typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

  // ros::NodeHandle nh(nh_, "image_thread");
  // ros::CallbackQueue queue;
  // nh.setCallbackQueue(&queue);

  // subscribe to cam msgs
  std::string cam0_topic(vk::param<std::string>(pnh_, "cam0_topic", "/cam0/image_raw"));
  std::string cam1_topic(vk::param<std::string>(pnh_, "cam1_topic", "/cam1/image_raw"));
  sub_img0 = pnh_.subscribe(cam0_topic, 10000, &FAR::FARInterface::subImage0CB, this, ros::TransportHints().tcpNoDelay());
  sub_img1 = pnh_.subscribe(cam1_topic, 10000, &FAR::FARInterface::subImage1CB, this, ros::TransportHints().tcpNoDelay());
  // image_transport::ImageTransport it(nh);
  // image_transport::SubscriberFilter sub0(it, cam0_topic, 1000, std::string("raw"));
  // image_transport::SubscriberFilter sub1(it, cam1_topic, 1000, std::string("raw"));
  // ExactSync sync_sub(ExactPolicy(1000), sub0, sub1);
  // sync_sub.registerCallback(boost::bind(&FAR::FARInterface::stereoCallback, this, _1, _2));

  // while(ros::ok() && !quit_)
  // {
  //   queue.callAvailable(ros::WallDuration(0.1));
  // }
}

void FARInterface::subImage0CB(const sensor_msgs::ImageConstPtr &img_msg)
{
  img_mutex_.lock();
  img0_buf_.push(img_msg);
  img_mutex_.unlock();
}

void FARInterface::subImage1CB(const sensor_msgs::ImageConstPtr &img_msg)
{
  img_mutex_.lock();
  img1_buf_.push(img_msg);
  img_mutex_.unlock();
}

} // namespace svo
