/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, WHILL, Inc.
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
#pragma once

namespace gpu_stereo_image_proc_visionworks {

struct VXStereoMatcherParams {
 public:
  enum class DisparityFiltering : int {
    None = 0,
    Bilateral = 1,
    WLS_LeftOnly = 2,
    WLS_LeftRight = 3
  };

  VXStereoMatcherParams()
      : downsample_log2(0),
        min_disparity(0),
        max_disparity(64),
        P1(8),
        P2(109),
        sad_win_size(5),
        ct_win_size(0),
        clip(31),
        max_diff(16),
        uniqueness_ratio(50),
        scanline_mask(NVX_SCANLINE_CROSS),
        flags(NVX_SGM_PYRAMIDAL_STEREO),
        filtering(DisparityFiltering::None),
        do_disparity_padding(true) {}

  void set_image_size(const cv::Size sz, int img_type) {
    _image_size.width = sz.width;
    _image_size.height = sz.height;
    _image_type = img_type;
  }

  const cv::Size image_size() const { return _image_size; }
  const int image_type() const { return _image_type; }

  // const cv::Size scaled_image_size() const {
  //   return cv::Size(_image_size.width >> downsample_log2,
  //                   _image_size.height >> downsample_log2);
  // }

  int downsample() const { return (1 << downsample_log2); }

  int disparity_padding() const {
    return do_disparity_padding ? max_disparity : 0;
  }

  int downsample_log2;
  int min_disparity;
  int max_disparity;

  int P1, P2, sad_win_size, ct_win_size, hc_win_size;
  int clip, max_diff, uniqueness_ratio, scanline_mask, flags;

  bool do_disparity_padding;

  DisparityFiltering filtering;

  struct BilateralFilterParams {
    BilateralFilterParams()
        : sigma_range(10),
          radius(3),
          num_iters(1),
          max_disc_threshold(0.2),
          edge_threshold(0.1) {
      ;
    }

    double sigma_range;
    int radius;
    int num_iters;
    double max_disc_threshold;
    double edge_threshold;

  } bilateral_filter_params;

  struct WLSFilterParams {
    WLSFilterParams()
        : lambda(8000),
          sigma_color(1.1),
          lrc_threshold(24),
          discontinuity_radius(2) {
      ;
    }

    double lambda, sigma_color;
    int lrc_threshold;
    int discontinuity_radius;

  } wls_filter_params;

  void dump() const {
    ROS_INFO("===================================");
    ROS_INFO("~~ Visionworks Matcher ~~");
    ROS_INFO("original img size : w %d, h %d", image_size().width,
             image_size().height);
    ROS_INFO("       Downsample : %d", downsample());
    ROS_INFO("       Uniqueness : %d", uniqueness_ratio);
    ROS_INFO("         Max Diff : %d", max_diff);
    ROS_INFO("            P1/P2 : P1 %d, P2 %d", P1, P2);
    ROS_INFO("         Win Size : SAD %d, CT %d, HC %d", sad_win_size,
             ct_win_size, hc_win_size);
    ROS_INFO("             Clip : %d", clip);
    ROS_INFO("     Min/Max Disp : min %d, max %d", min_disparity,
             max_disparity);
    ROS_INFO("   Scan type mask : 0x%02X", scanline_mask);
    ROS_INFO("            Flags : %02X", flags);
    ROS_INFO("        Filtering : %s", disparity_filter_as_string());
    ROS_INFO("     Edge padding : %s", (do_disparity_padding ? "YES" : "NO"));

    if (filtering == DisparityFiltering::WLS_LeftRight) {
      ROS_INFO_STREAM("WLS        Lambda : " << wls_filter_params.lambda);
      ROS_INFO_STREAM("WLS    SigmaColor : " << wls_filter_params.sigma_color);
      ROS_INFO_STREAM(
          "WLS LRC Threshold : " << wls_filter_params.lrc_threshold);
      ROS_INFO_STREAM(
          "WLS Discont. Radius : " << wls_filter_params.discontinuity_radius);
    } else if (filtering == DisparityFiltering::Bilateral) {
      ROS_INFO_STREAM(
          "Bilat       Sigma : " << bilateral_filter_params.sigma_range);
      ROS_INFO_STREAM("Bilat      Radius : " << bilateral_filter_params.radius);
      ROS_INFO_STREAM(
          "Bilat   Num iters : " << bilateral_filter_params.num_iters);
      ROS_INFO_STREAM("Bilat Max disc thr : "
                      << bilateral_filter_params.max_disc_threshold);
      ROS_INFO_STREAM(
          "Bilat    Edge thr : " << bilateral_filter_params.edge_threshold);
    }
    ROS_INFO("===================================");
  }

  bool valid() const {
    if (image_size().width == 0 || image_size().height == 0) return false;

    return true;
  }

  const char *disparity_filter_as_string() const {
    if (filtering == DisparityFiltering::None) {
      return "None";
    } else if (filtering == DisparityFiltering::Bilateral) {
      return "Bilateral";
    } else if (filtering == DisparityFiltering::WLS_LeftOnly) {
      return "WLS Left-only";
    } else if (filtering == DisparityFiltering::WLS_LeftRight) {
      return "WLS Left-Right";
    }

    return "(Unknown)";
  }

 private:
  cv::Size _image_size;
  int _image_type;

  // Validations from vx_sgbm_processor which we could re-implement

  // if (image_size.width % 4 != 0) {
  //   ROS_WARN("Image Width must be divisible by 4.");
  //   return false;
  // }

  // if (ratio < 0.0 || ratio > 100.0)
  //   return false;
};

}  // namespace gpu_stereo_image_proc_visionworks
