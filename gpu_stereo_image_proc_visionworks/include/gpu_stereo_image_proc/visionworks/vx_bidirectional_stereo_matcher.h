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

#include <opencv2/core.hpp>

#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"

namespace gpu_stereo_image_proc_visionworks {

class VXBidirectionalStereoMatcher : public VXStereoMatcher {
 public:
  VXBidirectionalStereoMatcher(const VXStereoMatcherParams &params);

  ~VXBidirectionalStereoMatcher();

  void compute(cv::InputArray left, cv::InputArray right) override;

  cv::Mat disparity() const override { return filter_output_; }

  cv::Mat confidenceMat() const { return confidence_; }

  cv::Mat RLDisparityMat() const {
    return vxImageToMatWrapper(flipped_rl_disparity_);
  }

 private:
  vx_image flipped_left_;
  vx_image flipped_right_;
  vx_image flipped_rl_disparity_;

  cv::Mat filter_output_, confidence_;

  // struct WLSParameters {
  //   double lambda;
  //   int lrc_threshold;
  // } _wls_params;

  // No default constructor
  VXBidirectionalStereoMatcher() = delete;

  // This class is non-copyable
  VXBidirectionalStereoMatcher(const VXBidirectionalStereoMatcher &) = delete;
  VXBidirectionalStereoMatcher &operator=(
      const VXBidirectionalStereoMatcher &) = delete;
};

}  // namespace gpu_stereo_image_proc_visionworks
