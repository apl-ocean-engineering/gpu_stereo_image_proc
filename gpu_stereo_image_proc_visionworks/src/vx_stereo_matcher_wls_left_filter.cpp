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

#include <ros/ros.h>

#include <NVX/nvx_opencv_interop.hpp>
#include <opencv2/stereo.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"

namespace gpu_stereo_image_proc_visionworks {

VXStereoMatcherWLSLeftFilter::VXStereoMatcherWLSLeftFilter(
    const VXStereoMatcherParams &params)
    : VXStereoMatcher(params) {
  ;
}

VXStereoMatcherWLSLeftFilter::~VXStereoMatcherWLSLeftFilter() { ; }

void VXStereoMatcherWLSLeftFilter::compute(cv::InputArray left,
                                           cv::InputArray right) {
  VXStereoMatcher::compute(left, right);

  // Copied from disparity_filters.cpp
  // https://github.com/opencv/opencv_contrib/blob/daaf645151b7afbafabfacf71ae2880cf6fc904e/modules/ximgproc/src/disparity_filters.cpp#L444C1-L444C107

  const int min_disp = params_.min_disparity;
  const int num_disp = (params_.max_disparity - params_.min_disparity);
  const int wsize = params_.sad_win_size;
  const int wsize2 = wsize / 2;

  cv ::Ptr<cv::ximgproc::DisparityWLSFilter> wls =
      cv::ximgproc::createDisparityWLSFilterGeneric(false);

  wls->setLambda(params_.wls_filter_params.lambda);
  wls->setLRCthresh(params_.wls_filter_params.lrc_threshold);
  wls->setDepthDiscontinuityRadius(
      params_.wls_filter_params.discontinuity_radius);
  wls->setSigmaColor(params_.wls_filter_params.sigma_color);

  nvx_cv::VXImageToCVMatMapper left_map(left_scaled_, 0, NULL, VX_READ_ONLY,
                                        VX_MEMORY_TYPE_HOST);
  nvx_cv::VXImageToCVMatMapper lr_disparity_map(
      disparity_, 0, NULL, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);

  const int border = params_.sad_win_size;
  const cv::Rect roi(border, 0, left_map.getMat().cols - 2 * border,
                     left_map.getMat().rows);

  wls->filter(lr_disparity_map.getMat(), left_map.getMat(), filter_output_,
              cv::Mat(), roi);
}

}  // namespace gpu_stereo_image_proc_visionworks
