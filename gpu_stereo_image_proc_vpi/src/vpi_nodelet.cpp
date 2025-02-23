/*********************************************************************
 *  Copyright (c) 2023 University of Washington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <chrono>
using namespace std::chrono;

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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <vpi/Version.h>

#include <memory>
#include <opencv2/calib3d/calib3d.hpp>

#include "code_timing/code_timing.h"
#include "gpu_stereo_image_proc/camera_info_conversions.h"
#include "gpu_stereo_image_proc/ros_topic_names.h"

// #include
// "gpu_stereo_image_proc/visionworks/vx_bidirectional_stereo_matcher.h"
#include "gpu_stereo_image_proc/nodelet_base.h"
#include "gpu_stereo_image_proc/vpi/vpi_stereo_matcher.h"
#include "gpu_stereo_image_proc_vpi/VPI_SGBMConfig.h"

namespace gpu_stereo_image_proc_vpi {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

using gpu_stereo_image_proc::RosTopic;
using gpu_stereo_image_proc::RosTopicNameMap;
using gpu_stereo_image_proc::RosTopicNameMap_V1;
using gpu_stereo_image_proc::RosTopicNameMap_V2;

class VPIDisparityNodelet : public gpu_stereo_image_proc::DisparityNodeletBase {
  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Publications
  ros::Publisher pub_disparity_, debug_lr_disparity_, debug_rl_disparity_;
  ros::Publisher debug_raw_disparity_, debug_disparity_mask_, pub_confidence_;
  ros::Publisher pub_depth_;

  ros::Publisher scaled_left_camera_info_, scaled_right_camera_info_;
  ros::Publisher scaled_left_rect_, scaled_right_rect_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef VPI_SGBMConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  std::unique_ptr<code_timing::CodeTiming> code_timing_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;

  VPIStereoMatcherParams params_;
  std::shared_ptr<VPIStereoMatcher> stereo_matcher_;
  bool debug_topics_;

  virtual void onInit();

  bool hasSubscribers() const override {
    return (pub_disparity_.getNumSubscribers() > 0) ||
           (pub_confidence_.getNumSubscribers() > 0) ||
           (pub_depth_.getNumSubscribers() > 0);
  }

  int downsample() const override { return params_.downsample(); }

  void imageCallback(const ImageConstPtr &l_image_msg,
                     const CameraInfoConstPtr &l_info_msg,
                     const ImageConstPtr &r_image_msg,
                     const CameraInfoConstPtr &r_info_msg) override;

  void configCb(Config &config, uint32_t level);

  // void bilateralConfigCb(BilateralFilterConfig &config, uint32_t level);
  // void wlsConfigCb(WLSFilterConfig &config, uint32_t level);

  bool update_stereo_matcher();

 public:
  VPIDisparityNodelet();
};

VPIDisparityNodelet::VPIDisparityNodelet() {}

void VPIDisparityNodelet::onInit() {
  DisparityNodeletBase::onInit();

  // Use the single-threaded model.   There are really only two callbacks:
  // config and new images.  Don't want to have to deconflict those
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  code_timing_.reset(new code_timing::CodeTiming(nh, "VPIDisparityNodelet"));

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f =
      boost::bind(&VPIDisparityNodelet::configCb, this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&DisparityNodeletBase::connectCb, this);

  std::string which_topic_names;
  private_nh.param("ros_topics_version", which_topic_names, std::string("v2"));

  RosTopicNameMap topic_names;
  if (which_topic_names == "v1") {
    NODELET_INFO("Using \"v1\" ROS topic names");
    topic_names = RosTopicNameMap_V1;
  } else {
    NODELET_INFO("Using \"v2\" ROS topic names");
    topic_names = RosTopicNameMap_V2;
  }

  // Make sure we don't enter connectCb() between advertising and assigning to
  // pub_disparity_
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_disparity_ = nh.advertise<DisparityImage>(
        topic_names[RosTopic::DownsampledDisparity], 1, connect_cb, connect_cb);

    pub_depth_ = nh.advertise<Image>(topic_names[RosTopic::DownsampledDepth], 1,
                                     connect_cb, connect_cb);

    pub_confidence_ =
        nh.advertise<Image>(topic_names[RosTopic::DownsampledConfidence], 1,
                            connect_cb, connect_cb);

    private_nh.param("debug", debug_topics_, false);
    if (debug_topics_) {
      ROS_INFO("Publishing debug topics");
      debug_lr_disparity_ = nh.advertise<DisparityImage>(
          topic_names[RosTopic::DownsampledDebugLRDisparity], 1);

      debug_rl_disparity_ = nh.advertise<DisparityImage>(
          topic_names[RosTopic::DownsampledDebugRLDisparity], 1);

      debug_raw_disparity_ = nh.advertise<DisparityImage>(
          topic_names[RosTopic::DownsampledDebugRawDisparity], 1, connect_cb,
          connect_cb);

      debug_disparity_mask_ = nh.advertise<Image>(
          topic_names[RosTopic::DownsampledDebugConfidenceMask], 1);
    }

    scaled_left_camera_info_ = nh.advertise<CameraInfo>(
        topic_names[RosTopic::DownsampledCameraInfoLeft], 1);
    scaled_right_camera_info_ = nh.advertise<CameraInfo>(
        topic_names[RosTopic::DownsampledCameraInfoRight], 1);

    scaled_left_rect_ =
        nh.advertise<Image>(topic_names[RosTopic::DownsampledRectifiedLeft], 1);
    scaled_right_rect_ = nh.advertise<Image>(
        topic_names[RosTopic::DownsampledRectifiedRight], 1);
  }
}

void VPIDisparityNodelet::imageCallback(const ImageConstPtr &l_image_msg,
                                        const CameraInfoConstPtr &l_info_msg,
                                        const ImageConstPtr &r_image_msg,
                                        const CameraInfoConstPtr &r_info_msg) {
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Create cv::Mat views in the two input buffers
  const cv::Mat l_image = cv_bridge::toCvShare(l_image_msg)->image;
  const cv::Mat r_image = cv_bridge::toCvShare(r_image_msg)->image;

  if ((l_image.size() != r_image.size()) || (l_image.rows == 0) ||
      (l_image.rows == 0) || (r_image.cols == 0) || (r_image.cols == 0))
    return;

  params_.set_image_size(cv::Size(l_image.cols, l_image.rows), l_image.type());
  if (stereo_matcher_) {
    if ((stereo_matcher_->params().image_size() != params_.image_size()) ||
        (stereo_matcher_->params().image_type() != params_.image_type())) {
      update_stereo_matcher();
    }
  } else {
    update_stereo_matcher();
  }

  // If you **still** don't have a stereo matcher, give up...
  if (!stereo_matcher_) return;

  // // Pull in some parameters as constants
  const int min_disparity = 0;
  const int max_disparity = stereo_matcher_->params().max_disparity;
  const int downsample = stereo_matcher_->params().downsample();
  const int border = stereo_matcher_->params().window_size / 2;

  const auto scaled_camera_info_l(scaleCameraInfo(l_info_msg, downsample));
  const auto scaled_camera_info_r(scaleCameraInfo(r_info_msg, downsample));

  image_geometry::StereoCameraModel scaled_model;
  scaled_model.fromCameraInfo(scaled_camera_info_l, scaled_camera_info_r);

  {
    auto timing = code_timing_->startBlock("disparity_calculation",
                                           "VPIDisparityNodelet");
    // Block matcher produces 16-bit signed (fixed point) disparity image
    stereo_matcher_->compute(l_image, r_image);
  }

  cv::Mat disparityS16 = stereo_matcher_->disparity();

  // double mmin, mmax;
  // cv::minMaxLoc(disparityS16, &mmin, &mmax);
  // NODELET_INFO_STREAM("Disparity min " << mmin << "; " << mmax);

  // if (debug_topics_) {
  //   DisparityImageGenerator raw_dg(l_image_msg, disparityS16, scaled_model,
  //                                  min_disparity, max_disparity, border);
  //   debug_raw_disparity_.publish(raw_dg.getDisparity());
  // }

  // cv::Mat confidence = stereo_matcher_->confidence();
  // cv_bridge::CvImage confidence_bridge(l_image_msg->header, "8UC1",
  //                                      confidence);
  // pub_confidence_.publish(confidence_bridge.toImageMsg());

  DisparityImageGenerator dg(scaled_model, min_disparity, max_disparity,
                             border);

  auto dgr(dg.generate(l_image_msg, disparityS16));

  pub_disparity_.publish(dgr.getDisparity());
  pub_depth_.publish(dgr.getDepth());

  scaled_left_camera_info_.publish(scaled_camera_info_l);
  scaled_right_camera_info_.publish(scaled_camera_info_r);

  {
    cv_bridge::CvImage left_rect_msg_bridge(l_image_msg->header, "mono8",
                                            stereo_matcher_->scaledLeftRect());
    scaled_left_rect_.publish(left_rect_msg_bridge.toImageMsg());
  }
  {
    cv_bridge::CvImage right_rect_msg_bridge(
        r_image_msg->header, "mono8", stereo_matcher_->scaledRightRect());
    scaled_right_rect_.publish(right_rect_msg_bridge.toImageMsg());
  }
}

void VPIDisparityNodelet::configCb(Config &config, uint32_t level) {
  // Settings for the nodelet itself
  // confidence_threshold_ = config.confidence_threshold;

  params_.window_size = config.correlation_window_size;
  params_.downsample_log2 = config.downsample;
  params_.quality = config.quality;
  params_.confidence_threshold = config.confidence_threshold;

#if NV_VPI_VERSION >= NV_VPI_MAKE_VERSION(1, 1, 0)
  params_.max_disparity = config.max_disparity;
#else
  if (config.max_disparity > 64) {
    ROS_WARN(
        "!!! Max disparity for this version of VPI is 64.   Limiting disparity "
        "to 64");
    params_.max_disparity = 64;
  } else {
    params_.max_disparity = config.max_disparity;
  }
#endif

  // // Disparity filter parameters
  // params_.disp_filter.sigma_range = config.sigma_range;
  // params_.disp_filter.radius = config.radius;
  // params_.disp_filter.num_iters = config.num_iters;
  // params_.disp_filter.max_disc_threshold = config.max_disc_threshold;
  // params_.disp_filter.edge_threshold = config.edge_threshold;

  // // WLS filter parameters
  // params_.wls_filter.lambda = config.lambda;
  // params_.wls_filter.lrc_threshold = config.lrc_threshold;

  update_stereo_matcher();
}

bool VPIDisparityNodelet::update_stereo_matcher() {
  // \todo For safety, should mutex this and imageCb
  ROS_WARN("Updating stereo_matcher");

  if (!params_.valid()) {
    ROS_WARN("Stereo matcher params are not valid...");
    return false;
  }

  params_.dump();
  ROS_WARN("Creating new stereo_matcher");
  // if (params_.filtering == VXStereoMatcherParams::Filtering_WLS_LeftRight) {
  //   ROS_INFO("Creating VXBidirectionalStereoMatcher");
  //   stereo_matcher_.reset(new VXBidirectionalStereoMatcher(params_));
  // } else {
  stereo_matcher_.reset(new VPIStereoMatcher(params_));
  //}
  return true;
}

}  // namespace gpu_stereo_image_proc_vpi

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gpu_stereo_image_proc_vpi::VPIDisparityNodelet,
                       nodelet::Nodelet)
