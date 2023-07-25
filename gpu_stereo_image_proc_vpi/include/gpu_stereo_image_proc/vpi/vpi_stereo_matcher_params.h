/*********************************************************************
 *  Copyright (c) 2023  University of Washington
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

namespace gpu_stereo_image_proc_vpi {

struct VPIStereoMatcherParams {
 public:
  enum DisparityFiltering_t {
    Filtering_None = 0,
    Filtering_Bilateral = 1,
    Filtering_WLS_LeftOnly = 2,
    Filtering_WLS_LeftRight = 3
  };

  VPIStereoMatcherParams()
      : downsample_log2(0),
        max_disparity(64),
        window_size(5),
        quality(6),
        confidence_threshold(32767),
        filtering(Filtering_None),
        do_disparity_padding(true) {}

  void set_image_size(const cv::Size sz, int img_type) {
    _image_size.width = sz.width;
    _image_size.height = sz.height;
    _image_type = img_type;
  }

  const int image_type() const { return _image_type; }

  // image_size is the **input** image size
  // scaled_image_size is the scaled-down image size
  // disparity_image_size is the scaled_down image size with disparity padding
  // (if used)
  const cv::Size image_size() const { return _image_size; }
  const cv::Size scaled_image_size() const {
    return cv::Size(_image_size.width >> downsample_log2,
                    _image_size.height >> downsample_log2);
  }

  const cv::Size disparity_image_size() const {
    const cv::Size ss(scaled_image_size());
    return cv::Size(ss.width + 2 * disparity_padding(), ss.height);
  }

  int downsample() const { return (1 << downsample_log2); }

  int disparity_padding() const {
#if NV_VPI_VERSION < VPI_VERSION_WITH_VIEWS
    // Older versions of VPI don't have views, we can't do disparity padding
    return 0;
#else
    return do_disparity_padding ? max_disparity : 0;
#endif
  }

  bool do_disparity_padding;
  int downsample_log2;
  int max_disparity, window_size;
  int quality, confidence_threshold;

  DisparityFiltering_t filtering;

  void dump() const {
    ROS_INFO("===================================");
    ROS_INFO("original img size : w %d, h %d", image_size().width,
             image_size().height);
    ROS_INFO("       Downsample : %d", downsample());
    ROS_INFO("      Window size : %d", window_size);
    ROS_INFO("          Quality : %d", quality);
    ROS_INFO(" Confidence threshold : %d", confidence_threshold);
    ROS_INFO("        Filtering : %s", disparity_filter_as_string());
#if NV_VPI_VERSION < VPI_VERSION_WITH_VIEWS
    ROS_INFO("     Edge padding : VPI too old, cannot do");
#else
    ROS_INFO("     Edge padding : %s", (do_disparity_padding ? "YES" : "NO"));
#endif
    ROS_INFO("===================================");
  }

  bool valid() const {
    if (image_size().width == 0 || image_size().height == 0) return false;

    return true;
  }

  const char *disparity_filter_as_string() const {
    if (filtering == Filtering_None) {
      return "None";
    } else if (filtering == Filtering_Bilateral) {
      return "Bilateral";
    } else if (filtering == Filtering_WLS_LeftOnly) {
      return "WLS Left-only";
    } else if (filtering == Filtering_WLS_LeftRight) {
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

}  // namespace gpu_stereo_image_proc_vpi
