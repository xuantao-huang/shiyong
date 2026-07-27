#include <ros/ros.h>
#include <estimator/far_nodelet.h>
#include <estimator/far_interface.h>
#include <pluginlib/class_list_macros.h>
#include <common/param_helper.h>

PLUGINLIB_EXPORT_CLASS(FAR::FARNodelet, nodelet::Nodelet)

namespace FAR {

FARNodelet::~FARNodelet()
{
  NODELET_INFO_STREAM("FAR quit");
  far_interface_->quit_ = true;
}

void FARNodelet::onInit()
{
  ros::NodeHandle nh(getNodeHandle());
  ros::NodeHandle pnh(getPrivateNodeHandle());

  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
  FAR::PipelineType type = FAR::PipelineType::kMono;
  if(vk::param<bool>(pnh, "pipeline_is_stereo", false))
    type = FAR::PipelineType::kStereo;

  far_interface_.reset(new FARInterface(type, nh, pnh));
  if(far_interface_->imu_handler_)
    far_interface_->subscribeImu();
  far_interface_->subscribeImage();
  far_interface_->subscribeRemoteKey();
}

} // namespace svo
