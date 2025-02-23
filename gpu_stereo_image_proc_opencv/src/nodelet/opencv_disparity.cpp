/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * This code is based on the vx_disparity.cpp from WHILL's gpu_stereo_image_proc
 * repo.  The original copyright is below, but for brevity the
 * contents of the license is moved to ./LICENSE
 *********************************************************************/

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <memory>
#include <opencv2/calib3d/calib3d.hpp>

#include "gpu_stereo_image_proc/camera_info_conversions.h"
#include "gpu_stereo_image_proc/msg_conversions.h"
#include "gpu_stereo_image_proc_opencv/OpenCVSGBMConfig.h"
// #include
// "gpu_stereo_image_proc/visionworks/vx_bidirectional_stereo_matcher.h"
// #include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"

namespace gpu_stereo_image_proc {
using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class OpenCVDisparityNodelet : public nodelet::Nodelet {
  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo>
      ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_disparity_, pub_scaled_disparity_, debug_lr_disparity_,
      debug_rl_disparity_;
  ros::Publisher scaled_left_camera_info_, scaled_right_camera_info_;
  ros::Publisher scaled_left_rect_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef gpu_stereo_image_proc::OpenCVSGBMConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;

  // VXStereoMatcherParams params_;
  // std::shared_ptr<VXStereoMatcherBase> stereo_matcher_;
  bool debug_topics_;

  virtual void onInit();

  void connectCb();

  void imageCb(const ImageConstPtr &l_image_msg,
               const CameraInfoConstPtr &l_info_msg,
               const ImageConstPtr &r_image_msg,
               const CameraInfoConstPtr &r_info_msg);

  void configCb(Config &config, uint32_t level);

  bool update_stereo_matcher();
};

void OpenCVDisparityNodelet::onInit() {
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx) {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                sub_l_image_, sub_l_info_,
                                                sub_r_image_, sub_r_info_));
    approximate_sync_->registerCallback(
        boost::bind(&OpenCVDisparityNodelet::imageCb, this, _1, _2, _3, _4));
  } else {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_,
                                    sub_l_info_, sub_r_image_, sub_r_info_));
    exact_sync_->registerCallback(
        boost::bind(&OpenCVDisparityNodelet::imageCb, this, _1, _2, _3, _4));
  }

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f =
      boost::bind(&OpenCVDisparityNodelet::configCb, this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&OpenCVDisparityNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to
  // pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_disparity_ =
      nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);

  pub_scaled_disparity_ = nh.advertise<DisparityImage>("scaled_disparity", 1,
                                                       connect_cb, connect_cb);

  private_nh.param("debug", debug_topics_, false);
  if (debug_topics_) {
    ROS_INFO("Publishing debug topics");
    debug_lr_disparity_ = nh.advertise<DisparityImage>("debug/lr_disparity", 1);

    debug_rl_disparity_ = nh.advertise<DisparityImage>("debug/rl_disparity", 1);
  }

  scaled_left_camera_info_ =
      nh.advertise<CameraInfo>("left/scaled_camera_info", 1);
  scaled_right_camera_info_ =
      nh.advertise<CameraInfo>("right/scaled_camera_info", 1);
  scaled_left_rect_ = nh.advertise<Image>("left/scaled_image_rect", 1);
}

// Handles (un)subscribing when clients (un)subscribe
void OpenCVDisparityNodelet::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if ((pub_disparity_.getNumSubscribers() == 0) &&
      (pub_scaled_disparity_.getNumSubscribers() == 0)) {
    sub_l_image_.unsubscribe();
    sub_l_info_.unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_.unsubscribe();
  } else if (!sub_l_image_.getSubscriber()) {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue
    // size.
    /// @todo Allow remapping left, right?
    image_transport::TransportHints hints("raw", ros::TransportHints(),
                                          getPrivateNodeHandle());
    sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
    sub_l_info_.subscribe(nh, "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
    sub_r_info_.subscribe(nh, "right/camera_info", 1);
  }
}

void OpenCVDisparityNodelet::imageCb(const ImageConstPtr &l_image_msg,
                                     const CameraInfoConstPtr &l_info_msg,
                                     const ImageConstPtr &r_image_msg,
                                     const CameraInfoConstPtr &r_info_msg) {
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image =
      cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;
  const cv::Mat_<uint8_t> r_image =
      cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;

  // params_.image_size = cv::Size(l_image.cols, l_image.rows);
  // if (stereo_matcher_) {
  //   const cv::Size image_size = stereo_matcher_->params().image_size;
  //   if (image_size.width != l_image.cols || image_size.height !=
  //   l_image.rows) {
  //     update_stereo_matcher();
  //   }
  // } else {
  //   update_stereo_matcher();
  // }

  // if (!stereo_matcher_)
  //   return;

  // Pull in some parameters as constants
  // const int min_disparity = stereo_matcher_->params().min_disparity;
  // const int max_disparity = stereo_matcher_->params().max_disparity;
  // const int shrink_scale = stereo_matcher_->params().shrink_scale;

  // const int border = stereo_matcher_->params().sad_win_size / 2;

  // {
  //   // Block matcher produces 16-bit signed (fixed point) disparity image
  //   cv::Mat_<int16_t> disparityS16;
  //   stereo_matcher_->compute(l_image, r_image, disparityS16);

  //   DisparityImagePtr disp_msg = disparityToDisparityImage(
  //       l_image_msg, disparityS16, model_, min_disparity * shrink_scale,
  //       max_disparity * shrink_scale, border * shrink_scale);

  //   pub_disparity_.publish(disp_msg);
  // }

  // {
  //   cv::Mat_<int16_t> scaledDisparityS16 =
  //       stereo_matcher_->scaledDisparityMat();
  //   DisparityImagePtr disp_msg = disparityToDisparityImage(
  //       l_image_msg, scaledDisparityS16, model_, min_disparity,
  //       max_disparity, border, shrink_scale);
  //   pub_scaled_disparity_.publish(disp_msg);
  // }

  // scaled_left_camera_info_.publish(scaleCameraInfo(l_info_msg,
  // shrink_scale));
  // scaled_right_camera_info_.publish(scaleCameraInfo(r_info_msg,
  // shrink_scale));

  // // Mildly inefficient but good enough...
  // cv::Mat scaledLeftRect;
  // cv::resize(l_image, scaledLeftRect, cv::Size(), 1.0 / shrink_scale,
  //            1.0 / shrink_scale);
  // cv_bridge::CvImage left_rect_msg_bridge(l_image_msg->header, "mono8",
  //                                         scaledLeftRect);
  // scaled_left_rect_.publish(left_rect_msg_bridge.toImageMsg());

  // if (debug_topics_) {
  //   // This is a copy, so only do it if necessary..
  //   cv::Mat scaledDisparity = stereo_matcher_->unfilteredDisparityMat();
  //   if (!scaledDisparity.empty()) {
  //     DisparityImagePtr lr_disp_msg = disparityToDisparityImage(
  //         l_image_msg, scaledDisparity, model_, min_disparity, max_disparity,
  //         border, shrink_scale);
  //     debug_lr_disparity_.publish(lr_disp_msg);
  //   }

  //   if (std::shared_ptr<VXBidirectionalStereoMatcher> bm =
  //           std::dynamic_pointer_cast<VXBidirectionalStereoMatcher>(
  //               stereo_matcher_)) {

  //     // This is a copy, so only do it if necessary..
  //     cv::Mat rlScaledDisparity = bm->scaledRLDisparityMat();
  //     if (!rlScaledDisparity.empty()) {
  //       DisparityImagePtr rl_disp_msg = disparityToDisparityImage(
  //           l_image_msg, rlScaledDisparity, model_, min_disparity,
  //           max_disparity, border, shrink_scale);
  //       debug_rl_disparity_.publish(rl_disp_msg);
  //     }
  //   }
  // }
}  // namespace gpu_stereo_image_proc

void OpenCVDisparityNodelet::configCb(Config &config, uint32_t level) {
  // Tweak all settings to be valid
  // config.correlation_window_size |= 0x1; // must be odd
  // config.max_disparity = (config.max_disparity / 4) * 4;
  // config.shrink_scale =
  //     static_cast<int>(pow(2, static_cast<int>(log2(config.shrink_scale))));

  // int scanline_mask = 0;
  // if (config.path_type == VXSGBM_SCANLINE_ALL) {
  //   scanline_mask = NVX_SCANLINE_ALL;
  // } else if (config.path_type == VXSGBM_SCANLINE_CROSS) {
  //   scanline_mask = NVX_SCANLINE_CROSS;
  // } else {
  //   if (config.SCANLINE_LEFT_RIGHT)
  //     scanline_mask |= NVX_SCANLINE_LEFT_RIGHT;
  //   if (config.SCANLINE_TOP_LEFT_BOTTOM_RIGHT)
  //     scanline_mask |= NVX_SCANLINE_TOP_LEFT_BOTTOM_RIGHT;
  //   if (config.SCANLINE_TOP_BOTTOM)
  //     scanline_mask |= NVX_SCANLINE_TOP_BOTTOM;
  //   if (config.SCANLINE_TOP_RIGHT_BOTTOM_LEFT)
  //     scanline_mask |= NVX_SCANLINE_TOP_RIGHT_BOTTOM_LEFT;
  //   if (config.SCANLINE_RIGHT_LEFT)
  //     scanline_mask |= NVX_SCANLINE_RIGHT_LEFT;
  //   if (config.SCANLINE_BOTTOM_RIGHT_TOP_LEFT)
  //     scanline_mask |= NVX_SCANLINE_BOTTOM_RIGHT_TOP_LEFT;
  //   if (config.SCANLINE_BOTTOM_TOP)
  //     scanline_mask |= NVX_SCANLINE_BOTTOM_TOP;
  //   if (config.SCANLINE_BOTTOM_LEFT_TOP_RIGHT)
  //     scanline_mask |= NVX_SCANLINE_BOTTOM_LEFT_TOP_RIGHT;
  // }

  // int flags = 0;
  // if (config.FILTER_TOP_AREA)
  //   flags |= NVX_SGM_FILTER_TOP_AREA;
  // if (config.PYRAMIDAL_STEREO)
  //   flags |= NVX_SGM_PYRAMIDAL_STEREO;

  // if (config.disparity_filter == VXSGBM_BilateralFilter) {
  //   ROS_INFO("Enabling bilateral filtering");
  //   params_.filtering = VXStereoMatcherParams::Filtering_Bilateral;
  //   // } else if (config.disparity_filter == VXSGBM_WLSFilter_LeftOnly) {
  //   //   ROS_INFO("Enabling Left-only WLS filtering");
  //   //   params_.filtering = VXStereoMatcherParams::Filtering_WLS_LeftOnly;
  // } else if (config.disparity_filter == VXSGBM_WLSFilter_LeftRight) {
  //   ROS_INFO("Enabling Left-Right WLS filtering");
  //   params_.filtering = VXStereoMatcherParams::Filtering_WLS_LeftRight;
  // } else {
  //   ROS_INFO("Disabling filtering");
  //   params_.filtering = VXStereoMatcherParams::Filtering_None;
  // }

  // // check stereo method
  // // Note: With single-threaded NodeHandle, configCb and imageCb can't be
  // called
  // // concurrently, so this is thread-safe.
  // params_.sad_win_size = config.correlation_window_size;
  // params_.min_disparity = config.min_disparity;
  // params_.max_disparity = config.max_disparity;
  // params_.uniqueness_ratio = config.uniqueness_ratio;
  // params_.P1 = config.P1;
  // params_.P2 = config.P2;
  // params_.max_diff = config.disp12MaxDiff;
  // params_.clip = config.bt_clip_value;
  // params_.ct_win_size = config.ct_win_size;
  // params_.hc_win_size = config.hc_win_size;
  // params_.flags = flags;
  // params_.scanline_mask = scanline_mask;
  // params_.shrink_scale = config.shrink_scale;

  update_stereo_matcher();
}

bool OpenCVDisparityNodelet::update_stereo_matcher() {
  // ROS_WARN("Updating stereo_matcher");

  // if (!params_.valid()) {
  //   ROS_WARN("No valid stereo matcher...");
  //   return false;
  // }

  // params_.dump();
  // ROS_WARN("Creating new stereo_matcher");
  // if (params_.filtering == VXStereoMatcherParams::Filtering_WLS_LeftRight) {
  //   ROS_INFO("Creating VXBidirectionalStereoMatcher");
  //   stereo_matcher_.reset(new VXBidirectionalStereoMatcher(params_));
  // } else {
  //   stereo_matcher_.reset(new VXStereoMatcher(params_));
  // }
  return true;
}

}  // namespace gpu_stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gpu_stereo_image_proc::OpenCVDisparityNodelet,
                       nodelet::Nodelet)
