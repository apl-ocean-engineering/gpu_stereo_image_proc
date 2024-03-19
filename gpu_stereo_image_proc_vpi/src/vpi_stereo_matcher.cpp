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

#include "gpu_stereo_image_proc/vpi/vpi_stereo_matcher.h"

#include <ros/ros.h>
#include <vpi/CUDAInterop.h>
#include <vpi/ImageFormat.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianFilter.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>

#include <iostream>
#include <opencv2/cudastereo.hpp>
#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>

#include "gpu_stereo_image_proc/vpi/vpi_status.h"

namespace gpu_stereo_image_proc_vpi {

VPIStatus makeGpuMatFPIWrapper(cv::cuda::GpuMat &mat, const cv::Size &sz,
                               int cvmode, VPIImageFormat fmt, VPIImage *img) {
  mat.create(sz, cvmode);

  VPIImageData imgData;
  memset(&imgData, 0, sizeof(imgData));
  imgData.bufferType = VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR;
  imgData.buffer.pitch.format = fmt;
  imgData.buffer.pitch.numPlanes = 1;
  imgData.buffer.pitch.planes[0].width = mat.cols;
  imgData.buffer.pitch.planes[0].height = mat.rows;
  imgData.buffer.pitch.planes[0].pitchBytes = mat.step;
  imgData.buffer.pitch.planes[0].data = mat.data;
  return vpiImageCreateWrapper(&imgData, 0, VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR,
                               img);
}

VPIStereoMatcher::VPIStereoMatcher(const VPIStereoMatcherParams &params)
    : params_(params) {
  // \todo ... fixed input type right now...
  VPIImageFormat input_format;  //= VPI_IMAGE_FORMAT_U8;
  int cv_format;
  if (params_.image_type() == CV_16UC1) {
    ROS_INFO("Using 16-bit input images");
    input_format = VPI_IMAGE_FORMAT_U16;
    cv_format = CV_16UC1;
  } else {
    ROS_INFO("Using 8-bit input images");
    input_format = VPI_IMAGE_FORMAT_U8;
    cv_format = CV_8UC1;
  }

  const cv::Size full_sz = params_.image_size();
  const cv::Size scaled_sz = params_.scaled_image_size();
  // const cv::Size disparity_sz = params_.disparity_image_size();

  const int backend = VPI_BACKEND_CUDA;

  VPI_CHECK_STATUS(vpiImageCreate(full_sz.width, full_sz.height, input_format,
                                  backend, &left_blurred_));

  VPI_CHECK_STATUS(vpiImageCreate(full_sz.width, full_sz.height, input_format,
                                  backend, &right_blurred_));

  ROS_INFO_STREAM(" Input image: " << full_sz.width << " x " << full_sz.height);

  ROS_INFO_STREAM("Scaled image: " << scaled_sz.width << " x "
                                   << scaled_sz.height);

  VPI_CHECK_STATUS(makeGpuMatFPIWrapper(
      left_scaled_gpu_mat_, cv::Size(scaled_sz.width, scaled_sz.height),
      cv_format, input_format, &left_scaled_));

  VPI_CHECK_STATUS(makeGpuMatFPIWrapper(
      right_scaled_gpu_mat_, cv::Size(scaled_sz.width, scaled_sz.height),
      cv_format, input_format, &right_scaled_));

  // // Outputs

  VPI_CHECK_STATUS(makeGpuMatFPIWrapper(
      disparity_gpu_mat_, cv::Size(scaled_sz.width, scaled_sz.height), CV_16SC1,
      VPI_IMAGE_FORMAT_S16, &disparity_));

  VPI_CHECK_STATUS(makeGpuMatFPIWrapper(
      disparity_filtered_gpu_mat_, cv::Size(scaled_sz.width, scaled_sz.height),
      CV_16SC1, VPI_IMAGE_FORMAT_S16, &disparity_filtered_));

  VPI_CHECK_STATUS(makeGpuMatFPIWrapper(
      confidence_gpu_mat_, cv::Size(scaled_sz.width, scaled_sz.height),
      CV_16UC1, VPI_IMAGE_FORMAT_U16, &confidence_));

  // ROS_INFO_STREAM("Max disparity: " << params_.max_disparity);
  VPIStereoDisparityEstimatorCreationParams create_params;
  vpiInitStereoDisparityEstimatorCreationParams(&create_params);
  create_params.maxDisparity = params_.max_disparity;

  VPI_CHECK_STATUS(vpiCreateStereoDisparityEstimator(
      backend, scaled_sz.width, scaled_sz.height, input_format, &create_params,
      &stereo_payload_));

  VPI_CHECK_STATUS(vpiStreamCreate(backend, &stream_));
}

VPIStereoMatcher::~VPIStereoMatcher() {
  vpiStreamDestroy(stream_);
  vpiPayloadDestroy(stereo_payload_);

  vpiImageDestroy(left_blurred_);
  vpiImageDestroy(right_blurred_);
  vpiImageDestroy(disparity_);
  vpiImageDestroy(disparity_filtered_);

  vpiImageDestroy(confidence_);
}

void VPIStereoMatcher::compute(cv::InputArray left_input,
                               cv::InputArray right_input) {
  VPIImage left, right;

  const int backend = VPI_BACKEND_CUDA;

  vpiImageUnlock(disparity_);

  VPI_CHECK_STATUS(
      vpiImageCreateWrapperOpenCVMat(left_input.getMat(), backend, &left));
  VPI_CHECK_STATUS(
      vpiImageCreateWrapperOpenCVMat(right_input.getMat(), backend, &right));

  const int gaussian_filter_size = 5;
  const float gaussian_filter_sigma = 1.7;

  // Blur the input images
  VPI_CHECK_STATUS(vpiSubmitGaussianFilter(
      stream_, backend, left, left_blurred_, gaussian_filter_size,
      gaussian_filter_size, gaussian_filter_sigma, gaussian_filter_sigma,
      VPI_BORDER_ZERO));
  VPI_CHECK_STATUS(vpiSubmitGaussianFilter(
      stream_, backend, right, right_blurred_, gaussian_filter_size,
      gaussian_filter_size, gaussian_filter_sigma, gaussian_filter_sigma,
      VPI_BORDER_ZERO));

  // Scale the input images
  VPI_CHECK_STATUS(vpiSubmitRescale(stream_, backend, left_blurred_,
                                    left_scaled_, VPI_INTERP_CATMULL_ROM,
                                    VPI_BORDER_ZERO, 0));
  VPI_CHECK_STATUS(vpiSubmitRescale(stream_, backend, right_blurred_,
                                    right_scaled_, VPI_INTERP_CATMULL_ROM,
                                    VPI_BORDER_ZERO, 0));

  VPIStereoDisparityEstimatorParams stereo_params;
  vpiInitStereoDisparityEstimatorParams(&stereo_params);
  stereo_params.confidenceType = VPI_STEREO_CONFIDENCE_ABSOLUTE;
  stereo_params.p1 = params_.p1;
  stereo_params.p2 = params_.p2;
  stereo_params.uniqueness = params_.uniqueness;
  stereo_params.confidenceThreshold = params_.confidence_threshold;

  // In the current generation of VPI, confidence is not
  // calculated.  This is fixed in later versions.
  VPI_CHECK_STATUS(vpiSubmitStereoDisparityEstimator(
      stream_, backend, stereo_payload_, left_scaled_, right_scaled_,
      disparity_, confidence_, &stereo_params));

  VPI_CHECK_STATUS(vpiStreamSync(stream_));

  vpiImageDestroy(left);
  vpiImageDestroy(right);

  if (params_.filtering == VPIStereoMatcherParams::Filtering_Bilateral) {
    const int nDisp = (params_.max_disparity - params_.min_disparity);

    cv::Ptr<cv::cuda::DisparityBilateralFilter> pCudaBilFilter =
        cv::cuda::createDisparityBilateralFilter(
            nDisp, params_.bilateral_filter_params.radius,
            params_.bilateral_filter_params.num_iters);

    pCudaBilFilter->apply(disparity_gpu_mat_, left_scaled_gpu_mat_,
                          disparity_filtered_gpu_mat_);

    // Output is now in disparity_filtered_ / disparity_filtered_gpu_mat_
  }
}

cv::Mat VPIStereoMatcher::confidence() {
  cv::Mat confidence_out;
  confidence_gpu_mat_.download(confidence_out);
  return confidence_out;
}

cv::Mat VPIStereoMatcher::disparity() {
  cv::Mat disp_out;

  if (params_.filtering == VPIStereoMatcherParams::Filtering_Bilateral) {
    disparity_filtered_gpu_mat_.download(disp_out);
  } else {
    disparity_gpu_mat_.download(disp_out);
  }

  return disp_out;
}

cv::Mat VPIStereoMatcher::scaledLeftRect() {
  cv::Mat left_out;
  left_scaled_gpu_mat_.download(left_out);
  return left_out;
}

cv::Mat VPIStereoMatcher::scaledRightRect() {
  cv::Mat right_out;
  right_scaled_gpu_mat_.download(right_out);
  return right_out;
}

}  // namespace gpu_stereo_image_proc_vpi
