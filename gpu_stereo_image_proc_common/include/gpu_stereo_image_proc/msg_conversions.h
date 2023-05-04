#pragma once

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

class DisparityImageGenerator {
  public: 
    stereo_msgs::DisparityImagePtr getDisparity();
    sensor_msgs::ImagePtr getDepth();

    DisparityImageGenerator(
      const sensor_msgs::ImageConstPtr &image, const cv::Mat_<int16_t> disparity16,
      const image_geometry::StereoCameraModel &model, int min_disparity,
      int max_disparity, int border);

  private: 
    stereo_msgs::DisparityImagePtr disparity;
};

stereo_msgs::DisparityImagePtr
disparityToDisparityImage(const sensor_msgs::ImageConstPtr &image,
                          const cv::Mat_<int16_t> disparity16,
                          const image_geometry::StereoCameraModel &model,
                          int min_disparity, int max_disparity, int border,
                          float shrink_scale = 1.0);

sensor_msgs::ImagePtr 
disparityImageToDepthImage(const stereo_msgs::DisparityImageConstPtr &disp_msg);