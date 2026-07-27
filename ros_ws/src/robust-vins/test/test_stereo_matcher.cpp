#include <tracker/direct/matcher.h>
#include <iostream>
#include <common/camera.h>
#include <ros/ros.h>
#include <common/param_helper.h>
#include <ros/package.h>
#include <common/transformation.h>
#include <opencv2/opencv.hpp>
#include <tracker/direct/feature_detection.h>
#include <tracker/direct/feature_detection_utils.h>
#include <opencv2/opencv.hpp>
#include <common/frame.h>

using namespace FAR;

DetectorOptions loadDetectorOptions(const ros::NodeHandle &pnh)
{
  DetectorOptions o;
  o.cell_size = vk::param<int>(pnh, "grid_size", 35);
  o.max_level = vk::param<int>(pnh, "n_pyr_levels", 3) - 1;
  o.threshold_primary = vk::param<int>(pnh, "detector_threshold_primary", 10);
  o.threshold_secondary = vk::param<int>(pnh, "detector_threshold_secondary", 200);
  o.threshold_shitomasi = vk::param<int>(pnh, "detector_threshold_shitomasi", 100);
  if (vk::param<bool>(pnh, "use_edgelets", true))
    o.detector_type = DetectorType::kFastGrad;
  else
    o.detector_type = DetectorType::kFast;
  return o;
}

FAR::CameraBundle::Ptr loadCameraFromYaml(const ros::NodeHandle &pnh)
{
  std::string calib_file = ros::package::getPath("far_vins") + "/param/calib/f4.yaml";
  CameraBundle::Ptr ncam = CameraBundle::loadFromYaml(calib_file);
  std::cout << "loaded " << ncam->numCameras() << " cameras";
  for (const auto &cam : ncam->getCameraVector())
    cam->printParameters(std::cout, "");
  return ncam;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "far_vins_node");
  std::cout << "Test Matcher1\n";
  ros::NodeHandle pnh("~");

  const std::string image0_path = ros::package::getPath("far_vins") + "/test/images/image0.png";
  const std::string image1_path = ros::package::getPath("far_vins") + "/test/images/image1.png";

  cv::Mat image0 = cv::imread(image0_path, 0);
  assert(!image0.empty());

  FAR::Matcher matcher;
  CameraBundle::Ptr ncam = loadCameraFromYaml(pnh);
  FAR::Transformation Tbc0 = ncam->get_T_C_B(0).inverse();
  FAR::Transformation Tbc1 = ncam->get_T_C_B(1).inverse();
  std::cout << "Tbc0:\n"
            << Tbc0 << std::endl;
  std::cout << "Tbc1:\n"
            << Tbc1 << std::endl;

  DetectorOptions feature_detector_options = loadDetectorOptions(pnh);
  AbstractDetector::Ptr feature_detecctor = feature_detection_utils::makeDetector(
      feature_detector_options, ncam->getCameraShared(0));

  FramePtr frame = std::make_shared<Frame>(ncam->getCameraShared(0), image0.clone(), 0, 3);
  feature_detecctor->detect(frame);

  cv::imshow("image", image0);
  cv::waitKey(0);
}