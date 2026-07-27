// Modification Note: 
// This file may have been modified by the authors of SchurVINS.
// (All authors of SchurVINS are with PICO department of ByteDance Corporation)
#include "estimator/far_node_base.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <common/logging.h>
#include <common/param_helper.h>

namespace FAR {

void FARNodeBase::initThirdParty(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InstallFailureSignalHandler();
  google::SetStderrLogging(google::GLOG_ERROR);
  // google::SetLogDestination(google::GLOG_INFO, "logs/");
  FLAGS_log_dir = "";

  ros::init(argc, argv, "far_vins_node");
}

FARNodeBase::FARNodeBase()
: node_handle_(), private_node_handle_("~"), type_(
    vk::param<bool>(private_node_handle_, "pipeline_is_stereo", false) ?
        FAR::PipelineType::kStereo : FAR::PipelineType::kMono),
        far_interface_(type_, node_handle_, private_node_handle_)
{
  if (far_interface_.imu_handler_)
  {
    far_interface_.subscribeImu();
  }
  sleep(1);
  far_interface_.subscribeImage();
  // far_interface_.subscribeRemoteKey();
  printFZ();
}

void FARNodeBase::run()
{
  ros::spin();
  far_interface_.quit_ = true;
}

}  // namespace svo_ros
