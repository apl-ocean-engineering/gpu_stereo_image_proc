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
#include <opencv2/cudastereo.hpp>

#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"

namespace gpu_stereo_image_proc_visionworks {

VXStereoMatcherBilateralFilter::VXStereoMatcherBilateralFilter(
    const VXStereoMatcherParams &params)
    : VXStereoMatcher(params) {
  ;
}

VXStereoMatcherBilateralFilter::~VXStereoMatcherBilateralFilter() { ; }

void VXStereoMatcherBilateralFilter::compute(cv::InputArray left,
                                             cv::InputArray right) {
  VXStereoMatcher::compute(left, right);

  const int nDisp = (params_.max_disparity - params_.min_disparity);
  const int radius = 3;
  const int iters = 1;

  nvx_cv::VXImageToCVMatMapper disparity_map(disparity_, 0, NULL, VX_READ_ONLY,
                                             NVX_MEMORY_TYPE_CUDA);
  nvx_cv::VXImageToCVMatMapper left_map(left_scaled_, 0, NULL, VX_READ_ONLY,
                                        NVX_MEMORY_TYPE_CUDA);

  cv::Ptr<cv::cuda::DisparityBilateralFilter> pCudaBilFilter =
      cv::cuda::createDisparityBilateralFilter(nDisp, radius, iters);

  pCudaBilFilter->apply(disparity_map.getGpuMat(), left_map.getGpuMat(),
                        g_filtered_);
}

}  // namespace gpu_stereo_image_proc_visionworks
