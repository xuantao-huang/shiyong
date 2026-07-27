#pragma once

#include "estimator/far_interface.h"

namespace FAR {

class FARNodeBase {
 public:
  // Initializes glog, gflags and ROS.
  static void initThirdParty(int argc, char **argv);

  FARNodeBase();

  void run();

 private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  FAR::PipelineType type_;

 public:
   FAR::FARInterface far_interface_;
};

}  // namespace svo_ros
