#pragma once

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

stereo_msgs::msg::DisparityImage::SharedPtr
disparityToDisparityImage(const sensor_msgs::msg::Image::ConstPtr &image,
                          const cv::Mat_<int16_t> disparity16,
                          const image_geometry::StereoCameraModel &model,
                          int min_disparity, int max_disparity, int border,
                          float shrink_scale = 1.0);