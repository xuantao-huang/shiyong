// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#include <estimator/initialization.h>

#include <random> // std::mt19937
#include <vector>

#include <common/frame.h>
#include <common/point.h>
#include <common/camera.h>
#include <common/container_helpers.h>
#include <tracker/direct/feature_detection.h>
#include <tracker/direct/feature_detection_utils.h>
#include <tracker/direct/feature_alignment.h>
#include <tracker/direct/matcher.h>
#include <estimator/stereo_triangulation.h>
// #include <svo/pose_optimizer.h>
#include <tracker/tracker/feature_tracker.h>
#include <tracker/tracker/feature_tracking_utils.h>
#include <vikit/cameras/ncamera.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>
#include <vikit/sample.h>
#include <opencv2/video/tracking.hpp> // for lucas kanade tracking
#include <opencv2/opencv.hpp> // for display


namespace FAR {

AbstractInitialization::AbstractInitialization(
    const InitializationOptions& init_options,
    const FeatureTrackerOptions& tracker_options,
    const DetectorOptions& detector_options,
    const CameraBundlePtr& cams)
  : options_(init_options)
{
  tracker_.reset(new FeatureTracker(tracker_options, detector_options, cams));
}

AbstractInitialization::~AbstractInitialization()
{}

void AbstractInitialization::reset()
{
  frames_ref_.reset();
  have_rotation_prior_ = false;
  have_translation_prior_ = false;
  have_depth_prior_ = false;
  tracker_->reset();
}

bool AbstractInitialization::trackFeaturesAndCheckDisparity(const FrameBundlePtr& frames)
{
  tracker_->trackFrameBundle(frames);
  std::vector<size_t> num_tracked;
  std::vector<double> disparity;
  tracker_->getNumTrackedAndDisparityPerFrame(
        options_.init_disparity_pivot_ratio, &num_tracked, &disparity);

  const size_t num_tracked_tot =
      std::accumulate(num_tracked.begin(), num_tracked.end(), 0u);
  const double avg_disparity =
      std::accumulate(disparity.begin(), disparity.end(), 0.0) / disparity.size();
  VLOG(3) << "Init: Tracked " << num_tracked_tot << " features with disparity = " << avg_disparity;
  if(num_tracked_tot < options_.init_min_features)
  {
    tracker_->resetActiveTracks();
    for(const FramePtr& frame : frames->frames_)
      frame->clearFeatureStorage();
    const size_t n = tracker_->initializeNewTracks(frames);
    VLOG(3) << "Init: New Tracks initialized = " << n;
    frames_ref_ = frames;
    R_ref_world_ = R_cur_world_;
    return false;
  }
  if(avg_disparity < options_.init_min_disparity)
    return false;
  return true;
}

InitResult HomographyInit::addFrameBundle(
    const FrameBundlePtr& frames_cur)
{
  // Track and detect features.
  if(!trackFeaturesAndCheckDisparity(frames_cur))
    return InitResult::kTracking;

  // Create vector of bearing vectors
  const FrameBundlePtr frames_ref = tracker_->getOldestFrameInTrack(0);
  const Frame& frame_ref = *frames_ref->at(0);
  const Frame& frame_cur = *frames_cur->at(0);
  FeatureMatches matches_cur_ref;
  feature_tracking_utils::getFeatureMatches(frame_cur, frame_ref, &matches_cur_ref);
  const size_t n = matches_cur_ref.size();
  Bearings f_cur(3, n);
  Bearings f_ref(3, n);
  for(size_t i = 0; i < n; ++i)
  {
    f_cur.col(i) = frame_cur.f_vec_.col(matches_cur_ref[i].first);
    f_ref.col(i) = frame_ref.f_vec_.col(matches_cur_ref[i].second);
  }

  // Compute model
  const vk::Homography H_cur_ref = vk::estimateHomography(
        f_cur, f_ref, frames_ref_->at(0)->getErrorMultiplier(),
        options_.reproj_error_thresh, options_.init_min_inliers);
  if(H_cur_ref.score < options_.init_min_inliers)
  {
    SVO_WARN_STREAM("Init Homography: Have " << H_cur_ref.score << "inliers. "
                    << options_.init_min_inliers << " inliers minimum required.");
    return InitResult::kFailure;
  }
  T_cur_from_ref_ = Transformation(Quaternion(H_cur_ref.R_cur_ref), H_cur_ref.t_cur_ref);

  // Triangulate
  if(initialization_utils::triangulateAndInitializePoints(
        frames_cur->at(0), frames_ref_->at(0), T_cur_from_ref_, options_.reproj_error_thresh,
        depth_at_current_frame_, options_.init_min_inliers, matches_cur_ref))
  {
    return InitResult::kSuccess;
  }

  // Restart
  tracker_->reset();
  frames_cur->at(0)->clearFeatureStorage();
  return InitResult::kTracking;
}



InitResult TwoPointInit::addFrameBundle(
    const FrameBundlePtr& frames_cur)
{
  return InitResult::kFailure;
}


InitResult FivePointInit::addFrameBundle(
    const FrameBundlePtr& frames_cur)
{
  return InitResult::kFailure;
}

StereoInit::StereoInit(
    const InitializationOptions& init_options,
    const FeatureTrackerOptions& tracker_options,
    const DetectorOptions& detector_options,
    const CameraBundlePtr& cams)
  : AbstractInitialization(init_options, tracker_options, detector_options, cams)
{
  StereoTriangulationOptions stereo_options;
  detector_ = feature_detection_utils::makeDetector(detector_options, cams->getCameraShared(0));
  stereo_.reset(new StereoTriangulation(stereo_options, detector_));
}

InitResult StereoInit::addFrameBundle(
    const FrameBundlePtr& frames)
{
  CHECK_EQ(frames->size(), 2u) << "StereoInit: Bundle has not two frames!";
  reset();
  frames_ref_ = frames;

  VLOG(20) << "FRAME 1" << std::endl << frames->at(0)->T_world_cam() <<std::endl
           << "FRAME 2" << std::endl << frames->at(1)->T_cam_world();

  stereo_->compute(frames->at(0), frames->at(1));
  if(frames->at(0)->numLandmarks() < options_.init_min_features)
    return InitResult::kFailure;
  return InitResult::kSuccess;
}

InitResult ArrayInitGeometric::addFrameBundle(
    const FrameBundlePtr& frames_cur)
{
  return InitResult::kFailure;
}

InitResult ArrayInitOptimization::addFrameBundle(
    const FrameBundlePtr& frames_cur)
{
  return InitResult::kFailure;
}


namespace initialization_utils {

bool triangulateAndInitializePoints(
    const FramePtr& frame_cur,
    const FramePtr& frame_ref,
    const Transformation& T_cur_ref,
    const double reprojection_threshold,
    const double depth_at_current_frame,
    const size_t min_inliers_threshold,
    AbstractInitialization::FeatureMatches& matches_cur_ref)
{
  Positions points_in_cur;
  initialization_utils::triangulatePoints(
        *frame_cur, *frame_ref, T_cur_ref, reprojection_threshold, matches_cur_ref, points_in_cur);

  if(matches_cur_ref.size() < min_inliers_threshold)
  {
    LOG(WARNING) << "Init WARNING: " <<min_inliers_threshold <<" inliers minimum required. "
                 << "Have only " << matches_cur_ref.size();
    return false;
  }

  // Scale 3D points to given scene depth and initialize Points
  initialization_utils::rescaleAndInitializePoints(
        frame_cur, frame_ref, matches_cur_ref, points_in_cur, T_cur_ref, depth_at_current_frame);

  return true;
}

void triangulatePoints(
    const Frame& frame_cur,
    const Frame& frame_ref,
    const Transformation& T_cur_ref,
    const double reprojection_threshold,
    AbstractInitialization::FeatureMatches& matches_cur_ref,
    Positions& points_in_cur)
{
  points_in_cur.resize(Eigen::NoChange, frame_cur.num_features_);
  const Transformation T_ref_cur = T_cur_ref.inverse();
  std::vector<size_t> outliers;
  size_t num_inliers = 0;
  const double angle_threshold =
      frame_cur.getAngleError(reprojection_threshold);
  for(size_t i = 0; i < matches_cur_ref.size(); ++i)
  {
    BearingVector f_cur = frame_cur.f_vec_.col(matches_cur_ref[i].first);
    BearingVector f_ref = frame_ref.f_vec_.col(matches_cur_ref[i].second);
    // TODO(cfo): should take reference to eigen
    const Position xyz_in_cur =
        vk::triangulateFeatureNonLin(
          T_cur_ref.getRotationMatrix(), T_cur_ref.getPosition(), f_cur, f_ref);

    const double e1 = angleError(f_cur, xyz_in_cur);
    const double e2 = angleError(f_ref, T_ref_cur * xyz_in_cur);
    if(e1 > angle_threshold || e2 > angle_threshold ||
       (frame_cur.cam()->getType() == Camera::Type::kPinhole &&
           xyz_in_cur.z() < 0.0))
    {
      outliers.push_back(i);
    }
    else
    {
      points_in_cur.col(num_inliers) = xyz_in_cur;
      ++num_inliers;
    }
  }
  if(!outliers.empty())
  {
    FAR::common::container_helpers::eraseIndicesFromVector(
        outliers, &matches_cur_ref);
  }
}

void rescaleAndInitializePoints(
    const FramePtr& frame_cur,
    const FramePtr& frame_ref,
    const AbstractInitialization::FeatureMatches& matches_cur_ref,
    const Positions& points_in_cur,
    const Transformation& T_cur_ref,
    const double depth_at_current_frame)
{
  // compute scale factor
  std::vector<double> depth_vec;
  for(size_t i = 0; i < matches_cur_ref.size(); ++i)
  {
    depth_vec.push_back(points_in_cur.col(i).norm());
  }
  CHECK_GT(depth_vec.size(), 1u);
  const double scene_depth_median = vk::getMedian(depth_vec);
  const double scale = depth_at_current_frame / scene_depth_median;

  // reset pose of current frame to have right scale
  frame_cur->T_f_w_ = T_cur_ref * frame_ref->T_f_w_;
  frame_cur->T_f_w_.getPosition() =
      - frame_cur->T_f_w_.getRotation().rotate(
        frame_ref->pos() + scale * (frame_cur->pos() - frame_ref->pos()));

  // Rescale 3D points and add to features
  Transformation T_world_cur = frame_cur->T_f_w_.inverse();
  for(size_t i = 0; i < matches_cur_ref.size(); ++i)
  {
    const Vector3d xyz_in_world = T_world_cur * (points_in_cur.col(i) * scale);
    const int point_id_cur = frame_cur->track_id_vec_(matches_cur_ref[i].first);
    const int point_id_ref = frame_ref->track_id_vec_(matches_cur_ref[i].second);
    CHECK_EQ(point_id_cur, point_id_ref);
    PointPtr new_point(new Point(point_id_cur, xyz_in_world));
    frame_cur->landmark_vec_.at(matches_cur_ref[i].first) = new_point;
    frame_ref->landmark_vec_.at(matches_cur_ref[i].second) = new_point;
    new_point->addObservation(frame_ref, matches_cur_ref[i].second);
    new_point->addObservation(frame_cur, matches_cur_ref[i].first);
  }

  SVO_INFO_STREAM("Init: Triangulated " << matches_cur_ref.size() << " points");
}

void displayFeatureTracks(
    const FramePtr& frame_cur,
    const FramePtr& frame_ref)
{
  cv::Mat img_rgb(frame_cur->img().size(), CV_8UC3);
  cv::cvtColor(frame_cur->img(), img_rgb, cv::COLOR_GRAY2RGB);

  /* TODO(cfo)
  for(size_t i=0; i<frame_cur->fts_.size(); ++i)
  {
    const FeaturePtr& ftr_cur = frame_cur->fts_.at(i);
    const FeaturePtr& ftr_ref = frame_ref->fts_.at(i);
    cv::line(img_rgb,
             cv::Point2f(ftr_cur->px[0], ftr_cur->px[1]),
             cv::Point2f(ftr_ref->px[0], ftr_ref->px[1]),
             cv::Scalar(0,0,255), 2);

  }
  */
  cv::imshow(frame_cur->cam_->getLabel().c_str(), img_rgb);
  cv::waitKey(0);
}

AbstractInitialization::UniquePtr makeInitializer(
    const InitializationOptions& init_options,
    const FeatureTrackerOptions& tracker_options,
    const DetectorOptions& detector_options,
    const CameraBundle::Ptr& cams)
{
  AbstractInitialization::UniquePtr initializer;
  switch(init_options.init_type)
  {
    case InitializerType::kHomography:
      initializer.reset(new HomographyInit(init_options, tracker_options, detector_options, cams));
      break;
    case InitializerType::kTwoPoint:
      initializer.reset(new TwoPointInit(init_options, tracker_options, detector_options, cams));
      break;
    case InitializerType::kFivePoint:
      initializer.reset(new FivePointInit(init_options, tracker_options, detector_options, cams));
      break;
    case InitializerType::kOneShot:
      // initializer.reset(new OneShotInit(init_options, tracker_options, detector_options, cams));
      break;
    case InitializerType::kStereo:
      initializer.reset(new StereoInit(init_options, tracker_options, detector_options, cams));
      break;
    case InitializerType::kArrayGeometric:
      initializer.reset(new ArrayInitGeometric(init_options, tracker_options, detector_options, cams));
      break;
    case InitializerType::kArrayOptimization:
      initializer.reset(new ArrayInitOptimization(init_options, tracker_options, detector_options, cams));
      break;
    default:
      LOG(FATAL) << "Initializer type not known.";
  }
  return initializer;
}

void copyBearingVectors(
    const Frame& frame_cur,
    const Frame& frame_ref,
    const AbstractInitialization::FeatureMatches& matches_cur_ref,
    AbstractInitialization::BearingVectors* f_cur,
    AbstractInitialization::BearingVectors* f_ref)
{
  CHECK_NOTNULL(f_cur);
  CHECK_NOTNULL(f_ref);
  f_cur->reserve(matches_cur_ref.size());
  f_ref->reserve(matches_cur_ref.size());
  for(size_t i = 0; i < matches_cur_ref.size(); ++i)
  {
    f_cur->push_back(frame_cur.f_vec_.col(matches_cur_ref[i].first));
    f_ref->push_back(frame_ref.f_vec_.col(matches_cur_ref[i].second));
  }
}

} // namespace initialization_utils

} // namespace svo
