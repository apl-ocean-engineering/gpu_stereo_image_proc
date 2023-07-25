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

#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"

#include <ros/ros.h>

#include <NVX/nvx_opencv_interop.hpp>
#include <iostream>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgproc.hpp>

#include "gpu_stereo_image_proc/visionworks/vx_conversions.h"

namespace gpu_stereo_image_proc_visionworks {

VXStereoMatcher::VXStereoMatcher(const VXStereoMatcherParams &params)
    : context_(nullptr),
      graph_(nullptr),
      left_image_(nullptr),
      right_image_(nullptr),
      left_scaled_(nullptr),
      right_scaled_(nullptr),
      disparity_(nullptr),
      left_scaler_(new VxGaussianImageScaler(params.downsample_log2,
                                             params.disparity_padding())),
      right_scaler_(new VxGaussianImageScaler(params.downsample_log2,
                                              params.disparity_padding())),
      params_(params) {
  vx_status status;

  context_ = vxCreateContext();
  VX_CHECK_STATUS(vxGetStatus((vx_reference)context_));

  graph_ = vxCreateGraph(context_);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)graph_));

  left_image_ = vxCreateImage(context_, params.image_size().width,
                              params.image_size().height, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)left_image_));

  right_image_ = vxCreateImage(context_, params.image_size().width,
                               params.image_size().height, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)right_image_));

  disparity_ =
      vxCreateImage(context_, params.scaled_image_size().width,
                    params.scaled_image_size().height, VX_DF_IMAGE_S16);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)disparity_));

  left_scaled_ = left_scaler_->addToGraph(context_, graph_, left_image_);
  right_scaled_ = right_scaler_->addToGraph(context_, graph_, right_image_);

  vx_node sgm_node = nvxSemiGlobalMatchingNode(
      graph_, left_scaled_, right_scaled_, disparity_, params.min_disparity,
      params.max_disparity, params.P1, params.P2, params.sad_win_size,
      params.ct_win_size, params.hc_win_size, params.clip, params.max_diff,
      params.uniqueness_ratio, params.scanline_mask, params.flags);
  VX_CHECK_STATUS(vxVerifyGraph(graph_));
  vxReleaseNode(&sgm_node);
}

VXStereoMatcher::~VXStereoMatcher() {
  vxReleaseImage(&left_image_);
  vxReleaseImage(&right_image_);
  vxReleaseImage(&disparity_);

  // Explicitly delete scalers before the graph is deleted
  left_scaler_.reset(nullptr);
  right_scaler_.reset(nullptr);

  vxReleaseGraph(&graph_);
  vxReleaseContext(&context_);
}

void VXStereoMatcher::compute(cv::InputArray left, cv::InputArray right) {
  //  left_image_ = 	nvx_cv::createVXImageFromCVMat(context_, left.getMat());
  //  right_image_ = 	nvx_cv::createVXImageFromCVMat(context_,
  //  right.getMat());
  copy_to_vx_image(left, left_image_);
  copy_to_vx_image(right, right_image_);

  const auto status = vxProcessGraph(graph_);
  ROS_ASSERT(status == VX_SUCCESS);

  if (params_.filtering == VXStereoMatcherParams::Filtering_Bilateral) {
    const int nDisp = (params_.max_disparity - params_.min_disparity);
    const int radius = 3;
    const int iters = 1;

    nvx_cv::VXImageToCVMatMapper disparity_map(
        disparity_, 0, NULL, VX_READ_ONLY, NVX_MEMORY_TYPE_CUDA);
    nvx_cv::VXImageToCVMatMapper left_map(left_scaled_, 0, NULL, VX_READ_ONLY,
                                          NVX_MEMORY_TYPE_CUDA);

    cv::Ptr<cv::cuda::DisparityBilateralFilter> pCudaBilFilter =
        cv::cuda::createDisparityBilateralFilter(nDisp, radius, iters);

    pCudaBilFilter->apply(disparity_map.getGpuMat(), left_map.getGpuMat(),
                          g_filtered_);
  }
}

}  // namespace gpu_stereo_image_proc_visionworks
