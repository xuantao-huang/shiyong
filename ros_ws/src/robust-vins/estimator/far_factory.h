#pragma once

#include <memory>
#include <ros/ros.h>
#include <common/camera_fwd.h>

namespace FAR {

// forward declarations
class ImuHandler;
// class LoopClosing;
// class GlobalMap;
class FrameHandlerMono;
class FrameHandlerStereo;
// class FrameHandlerArray;
// class FrameHandlerDenseMono;

namespace factory {

/// Get IMU Handler.
std::shared_ptr<ImuHandler> getImuHandler(
    const ros::NodeHandle& pnh);

/// Factory for Mono-SVO.
std::shared_ptr<FrameHandlerMono> makeMono(
    const ros::NodeHandle& pnh,
    const CameraBundlePtr& cam = nullptr);

/// Factory for Stereo-SVO.
std::shared_ptr<FrameHandlerStereo> makeStereo(
    const ros::NodeHandle& pnh,
    const CameraBundlePtr& cam = nullptr);

} // namespace factory
} // namespace mono
