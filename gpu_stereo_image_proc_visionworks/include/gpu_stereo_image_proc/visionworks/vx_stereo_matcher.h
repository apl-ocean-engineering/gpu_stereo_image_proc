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

#include <NVX/nvx.h>
#include <VX/vx.h>
#include <VX/vxu.h>
#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

// #include <memory>
// #include <opencv2/core.hpp>

#include "gpu_stereo_image_proc/visionworks/vx_conversions.h"
#include "gpu_stereo_image_proc/visionworks/vx_image_scaler.h"
#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher_params.h"

namespace gpu_stereo_image_proc_visionworks {

class VXStereoMatcher {
 public:
  VXStereoMatcher(const VXStereoMatcherParams &params);

  virtual ~VXStereoMatcher();

  virtual void compute(cv::InputArray left, cv::InputArray right);

  const VXStereoMatcherParams &params() const { return params_; }

  cv::Mat unfilteredDisparityMat() const {
    return vxImageToMatWrapper(disparity_);
  }

  cv::Mat scaledLeftRect() const { return vxImageToMatWrapper(left_scaled_); }

  virtual cv::Mat disparity() const { return vxImageToMatWrapper(disparity_); }

 protected:
  cv::Size leftScaledSize() const {
    cv::Size disparity_size;
    VX_CHECK_STATUS(vxQueryImage(left_scaled_, VX_IMAGE_WIDTH,
                                 &disparity_size.width, sizeof(vx_uint32)));
    VX_CHECK_STATUS(vxQueryImage(left_scaled_, VX_IMAGE_HEIGHT,
                                 &disparity_size.height, sizeof(vx_uint32)));
    return disparity_size;
  }

  vx_context context_;
  vx_graph graph_;

  // Input images
  vx_image left_image_;
  vx_image right_image_;

  // Scaled images (equal to {left|right}_image_ if not scaling)
  vx_image left_scaled_;
  vx_image right_scaled_;

  // Output images
  vx_image disparity_;

  VXStereoMatcherParams params_;

  std::unique_ptr<VxImageScaler> left_scaler_, right_scaler_;

  VXStereoMatcher() = delete;

  // noncopyable
  VXStereoMatcher(const VXStereoMatcher &) = delete;
  VXStereoMatcher &operator=(const VXStereoMatcher &) = delete;
};

class VXStereoMatcherBilateralFilter : public VXStereoMatcher {
 public:
  VXStereoMatcherBilateralFilter(const VXStereoMatcherParams &params);
  virtual ~VXStereoMatcherBilateralFilter();

  void compute(cv::InputArray left, cv::InputArray right) override;

  cv::Mat disparity() const override {
    // I suspect this is inefficient...
    cv::Mat out;
    g_filtered_.download(out);
    return out;
  }

 protected:
 private:
  // GpuMat which stores the result **if** filtering is enabled
  cv::cuda::GpuMat g_filtered_;

  VXStereoMatcherBilateralFilter() = delete;
  VXStereoMatcherBilateralFilter(const VXStereoMatcherBilateralFilter &) =
      delete;
  VXStereoMatcherBilateralFilter &operator=(
      const VXStereoMatcherBilateralFilter &) = delete;
};

class VXStereoMatcherWLSLeftFilter : public VXStereoMatcher {
 public:
  VXStereoMatcherWLSLeftFilter(const VXStereoMatcherParams &params);
  virtual ~VXStereoMatcherWLSLeftFilter();

  void compute(cv::InputArray left, cv::InputArray right) override;

  cv::Mat disparity() const override {
    // WLS filter runs on the CPU (for now), so it's output is natively on CPU
    return filter_output_;
  }

 protected:
  cv::Mat filter_output_;

  VXStereoMatcherWLSLeftFilter() = delete;
  VXStereoMatcherWLSLeftFilter(const VXStereoMatcherWLSLeftFilter &) = delete;
  VXStereoMatcherWLSLeftFilter &operator=(
      const VXStereoMatcherWLSLeftFilter &) = delete;
};

}  // namespace gpu_stereo_image_proc_visionworks
