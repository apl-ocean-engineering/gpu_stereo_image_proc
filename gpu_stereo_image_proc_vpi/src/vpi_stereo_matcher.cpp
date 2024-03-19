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

VPIStatus makeGpuMatFPIWrapper(cv::Mat &mat, cv::Size &sz,
                               int cvmode VPIImageFormat fmt, VPIImage *img) {
  mat = cv::GpuMat(sz, cvmode);

  // Sample code for wrapper a VPIImage around contents of a cvGpuImage
  // +        cv::cuda::GpuMat cvGpuImage(cvImage);
  // +
  // +        VPIImageData imgData;
  // +        memset(&imgData, 0, sizeof(imgData));
  // +        imgData.format               = VPI_IMAGE_FORMAT_U8;
  // +        imgData.numPlanes            = 1;
  // +        imgData.planes[0].width      = cvGpuImage.cols;
  // +        imgData.planes[0].height     = cvGpuImage.rows;
  // +        imgData.planes[0].pitchBytes = cvGpuImage.step;
  // +        imgData.planes[0].data       = cvGpuImage.data;
  // +        CHECK_STATUS(vpiImageCreateCUDAMemWrapper(&imgData, 0, &image));

  VPIImageData imgData;
  memset(&imgData, 0, sizeof(imgData));
  imgData.format = fmt;
  imgData.numPlanes = 1;
  imgData.planes[0].width = mat.cols;
  imgData.planes[0].height = mat.rows;
  imgData.planes[0].pitchBytes = mat.step;
  imgData.planes[0].data = mat.data;
  return vpiImageCreateCUDAMemWrapper(&imgData, 0, mat);
}

VPIStereoMatcher::VPIStereoMatcher(const VPIStereoMatcherParams &params)
    : params_(params) {
  // \todo ... fixed input type right now...
  VPIImageFormat input_format;  //= VPI_IMAGE_FORMAT_U8;
  int cv_format;
  if (params_.image_type() == CV_16UC1) {
    ROS_INFO("Using 16-bit input images");
    input_format = VPI_IMAGE_FORMAT_U16;
    cv_format = cv::CV_16UC1;
  } else {
    ROS_INFO("Using 8-bit input images");
    input_format = VPI_IMAGE_FORMAT_U8;
    cv_format = cv::CV_8UC1
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
      cv_format, input_format, left_scaled_));

  VPI_CHECK_STATUS(makeGpuMatFPIWrapper(
      right_scaled_gpu_mat_, cv::Size(scaled_sz.width, scaled_sz.height),
      cv_format, input_format, right_scaled_));

  // VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
  //                                 input_format, backend, &left_scaled_));

  // VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
  //                                 input_format, backend, &right_scaled_));

  // // Outputs

  // VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
  //                                 VPI_IMAGE_FORMAT_S16, backend,
  //                                 &disparity_));

  // VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
  //                                 VPI_IMAGE_FORMAT_S16, backend,
  //                                 &disparity_filtered_));

  // VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
  //                                 VPI_IMAGE_FORMAT_U16, backend,
  //                                 &confidence_));

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

  ///
  ///
  ///

  // VPIStereoDisparityEstimatorCreationParams create_params;
  // create_params.maxDisparity = 64; //params.max_disparity;

  //   int32_t stereo_width, stereo_height;
  //   VPIImageFormat stereo_format;
  //   VPI_CHECK_STATUS(vpiImageGetSize( left_scaled_, &stereo_width,
  //   &stereo_height )); VPI_CHECK_STATUS(vpiImageGetFormat(left_scaled_,
  //   &stereo_format));
  // ROS_INFO_STREAM_THROTTLE(1, "     Left is " << stereo_width << " x " <<
  // stereo_height << " ; format " << vpiImageFormatGetName(stereo_format));

  //   VPI_CHECK_STATUS(vpiImageGetSize( right_scaled_, &stereo_width,
  //   &stereo_height )); VPI_CHECK_STATUS(vpiImageGetFormat(right_scaled_,
  //   &stereo_format));
  // ROS_INFO_STREAM_THROTTLE(1, "    Right is " << stereo_width << " x " <<
  // stereo_height << " ; format " << vpiImageFormatGetName(stereo_format));

  //   VPI_CHECK_STATUS(vpiImageGetSize( disparity_, &stereo_width,
  //   &stereo_height )); VPI_CHECK_STATUS(vpiImageGetFormat(disparity_,
  //   &stereo_format));
  // ROS_INFO_STREAM_THROTTLE(1, "Disparity is " << stereo_width << " x " <<
  // stereo_height << " ; format " << vpiImageFormatGetName(stereo_format));

  // VPIPayload stereo_payload;
  // VPI_CHECK_STATUS(vpiCreateStereoDisparityEstimator(
  //     VPI_BACKEND_CUDA, stereo_width, stereo_height, stereo_format,
  //     &create_params, &stereo_payload));

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
    // const int radius = 3;
    // const int iters = 1;

    // cv::Mat disp_export, disp_filtered_export, left_export;

    // VPIImageData disp_data, disp_filtered_data, disp_left_scaled;
    // VPI_CHECK_STATUS(vpiImageLockData(disparity_, VPI_LOCK_READ,
    //                                   VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR,
    //                                   &disp_data));
    // VPI_CHECK_STATUS(vpiImageDataExportOpenCVMat(disp_data, &disp_export));

    // VPI_CHECK_STATUS(vpiImageLockData(disparity_filtered_,
    // VPI_LOCK_READ_WRITE,
    //                                   VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR,
    //                                   &disp_filtered_data));
    // VPI_CHECK_STATUS(
    //     vpiImageDataExportOpenCVMat(disp_filtered_data,
    //     &disp_filtered_export));

    // VPI_CHECK_STATUS(vpiImageLockData(left_scaled_, VPI_LOCK_READ,
    //                                   VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR,
    //                                   &disp_left_scaled));
    // VPI_CHECK_STATUS(
    //     vpiImageDataExportOpenCVMat(disp_left_scaled, &left_export));

    // cv::Ptr<cv::cuda::DisparityBilateralFilter> pCudaBilFilter =
    //     cv::cuda::createDisparityBilateralFilter(
    //         nDisp, params_.bilateral_filter_params.radius,
    //         params_.bilateral_filter_params.num_iters);

    // pCudaBilFilter->apply(disp_export, left_export, disp_filtered_export);

    // vpiImageUnlock(disparity_);
    // vpiImageUnlock(disparity_filtered_);
    // vpiImageUnlock(left_scaled_);

    disparity_output_ = disparity_;  // filtered_;
  } else {
    disparity_output_ = disparity_;
  }
}

cv::Mat VPIStereoMatcher::confidence() {
  cv::Mat confidence_export, confidence_out;
  VPIImageData confidence_data;
  VPI_CHECK_STATUS(vpiImageLockData(confidence_, VPI_LOCK_READ,
                                    VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR,
                                    &confidence_data));
  VPI_CHECK_STATUS(
      vpiImageDataExportOpenCVMat(confidence_data, &confidence_export));

  confidence_export.copyTo(confidence_out);
  vpiImageUnlock(confidence_);

  return confidence_out;
}

cv::Mat VPIStereoMatcher::disparity() {
  VPIImageData disp_data;
  VPI_CHECK_STATUS(vpiImageLockData(disparity_output_, VPI_LOCK_READ,
                                    VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR,
                                    &disp_data));

  cv::Mat disp_export, disp_out;
  VPI_CHECK_STATUS(vpiImageDataExportOpenCVMat(disp_data, &disp_export));

  //   double mmin, mmax;
  //   cv::minMaxLoc(disp_export, &mmin, &mmax);
  //   ROS_INFO_STREAM_THROTTLE(1, "Disparity min " << mmin << "; max = " <<
  //   mmax);

  disp_export.copyTo(disp_out);
  vpiImageUnlock(disparity_output_);

  return disp_out;
}

cv::Mat VPIStereoMatcher::scaledLeftRect() {
  VPIImageData left_data;
  VPI_CHECK_STATUS(vpiImageLockData(left_scaled_, VPI_LOCK_READ,
                                    VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR,
                                    &left_data));

  cv::Mat left_export, left_out;
  VPI_CHECK_STATUS(vpiImageDataExportOpenCVMat(left_data, &left_export));

  left_export.copyTo(left_out);
  vpiImageUnlock(left_scaled_);

  return left_out;
}

cv::Mat VPIStereoMatcher::scaledRightRect() {
  VPIImageData right_data;
  VPI_CHECK_STATUS(vpiImageLockData(right_scaled_, VPI_LOCK_READ,
                                    VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR,
                                    &right_data));

  cv::Mat right_export, right_out;
  VPI_CHECK_STATUS(vpiImageDataExportOpenCVMat(right_data, &right_export));

  right_export.copyTo(right_out);
  vpiImageUnlock(right_scaled_);

  return right_out;
}

}  // namespace gpu_stereo_image_proc_vpi
