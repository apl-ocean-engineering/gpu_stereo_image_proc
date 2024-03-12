#pragma once

#include <map>
#include <string>

namespace gpu_stereo_image_proc {

enum class RosTopic : unsigned int {
  DownsampledDisparity,
  DownsampledDisparityImage,
  DownsampledDepth,
  DownsampledConfidence,
  DownsampledRectifiedLeft,
  DownsampledRectifiedRight,
  DownsampledCameraInfoLeft,
  DownsampledCameraInfoRight,
  DownsampledDebugLRDisparity,
  DownsampledDebugRLDisparity,
  DownsampledDebugRawDisparity,
  DownsampledDebugConfidenceMask,

};

typedef std::map<RosTopic, std::string> RosTopicNameMap;

RosTopicNameMap RosTopicNameMap_V1 = {
    {RosTopic::DownsampledDisparity, "stereo/disparity"},
    {RosTopic::DownsampledDisparityImage, "stereo/disparity/image"},
    {RosTopic::DownsampledConfidence, "stereo/confidence"},
    {RosTopic::DownsampledDepth, "stereo/depth"},
    {RosTopic::DownsampledRectifiedLeft, "stereo/left/scaled_image_rect"},
    {RosTopic::DownsampledRectifiedRight, "stereo/right/scaled_image_rect"},
    {RosTopic::DownsampledCameraInfoLeft, "stereo/left/scaled_camera_info"},
    {RosTopic::DownsampledCameraInfoRight, "stereo/right/scaled_camera_info"},
    {RosTopic::DownsampledDebugLRDisparity, "stereo/debug/lr_disparity"},
    {RosTopic::DownsampledDebugRLDisparity, "stereo/debug/rl_disparity"},
    {RosTopic::DownsampledDebugRawDisparity, "stereo/debug/raw_disparity"},
    {RosTopic::DownsampledDebugConfidenceMask, "stereo/debug/confidence_mask"},
};

RosTopicNameMap RosTopicNameMap_V2 = {
    {RosTopic::DownsampledDisparity, "downsampled/disparity"},
    {RosTopic::DownsampledDisparityImage, "downsampled/disparity/image"},
    {RosTopic::DownsampledConfidence, "downsampled/confidence"},
    {RosTopic::DownsampledDepth, "downsampled/depth"},
    {RosTopic::DownsampledRectifiedLeft, "downsampled/left/image_rect"},
    {RosTopic::DownsampledRectifiedRight, "downsampled/right/image_rect"},
    {RosTopic::DownsampledCameraInfoLeft, "downsampled/left/camera_info"},
    {RosTopic::DownsampledCameraInfoRight, "downsampled/right/camera_info"},
    {RosTopic::DownsampledDebugLRDisparity, "downsampled/debug/lr_disparity"},
    {RosTopic::DownsampledDebugRLDisparity, "downsampled/debug/rl_disparity"},
    {RosTopic::DownsampledDebugRawDisparity, "downsampled/debug/raw_disparity"},
    {RosTopic::DownsampledDebugConfidenceMask,
     "downsampled/debug/confidence_mask"},

};
}  // namespace gpu_stereo_image_proc
