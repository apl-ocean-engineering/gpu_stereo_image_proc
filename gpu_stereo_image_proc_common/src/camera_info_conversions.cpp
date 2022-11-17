#include <gpu_stereo_image_proc/camera_info_conversions.h>
// #include <ros/ros.h>

sensor_msgs::msg::CameraInfo::SharedPtr
scaleCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam,
                float downsample) {

  sensor_msgs::msg::CameraInfo::SharedPtr out =
      std::make_shared<sensor_msgs::msg::CameraInfo>();

  out->header = cam->header;

  out->width = cam->width / downsample;
  out->height = cam->height / downsample;

  // Distortion parameters aaren't meaningful after scaling
  //  out->distortion_model
  //  out->D

  out->k[0] = cam->k[0] / downsample;
  out->k[2] = cam->k[2] / downsample;
  out->k[4] = cam->k[4] / downsample;
  out->k[5] = cam->k[5] / downsample;
  out->k[8] = 1;

  out->r = cam->r;

  out->p[0] = cam->p[0] / downsample;
  out->p[2] = cam->p[2] / downsample;
  out->p[3] = cam->p[3] / downsample;

  out->p[5] = cam->p[5] / downsample;
  out->p[6] = cam->p[6] / downsample;
  out->p[7] = cam->p[7] / downsample;

  out->p[10] = 1;

  return out;
}
