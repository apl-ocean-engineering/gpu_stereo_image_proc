#pragma once

#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>

sensor_msgs::msg::CameraInfo::SharedPtr
scaleCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam,
                float downsample = 1.0);
