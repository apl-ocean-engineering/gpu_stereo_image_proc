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

#include "gpu_stereo_image_proc/visionworks/vx_image_scaler.h"

#include <iostream>

#include "gpu_stereo_image_proc/visionworks/vx_conversions.h"

namespace gpu_stereo_image_proc_visionworks {

VxImageScaler::VxImageScaler(unsigned int downsample_log2,
                             unsigned int disparity_padding)
    : downsample_log2_(downsample_log2),
      disparity_padding_(disparity_padding) {}

VxGaussianImageScaler::VxGaussianImageScaler(unsigned int downsample_log2,
                                             unsigned int disparity_padding)
    : VxImageScaler(downsample_log2, disparity_padding), images_() {
  assert(downsample_log2_ >= 0);

  images_.resize(downsample_log2_);
}

// This fills in the last two entries in images_ (using the size hint)
// if disparity_padding == 0, they are the same
// if disparity_padding > 0, the last layer is the padded size
//                           and the second-to-last-layer is an ROI into the
//                           last layer
//
// In any case the last layer can be used as output_image_
void VxGaussianImageScaler::addImage(const size_t idx, vx_context context,
                                     unsigned int width, unsigned int height,
                                     vx_df_image format) {
  assert((idx + 1) < images_.size());

  // Create next layer
  if (idx == downsample_log2_) {
    // std::cout << "Building output at layer " << idx+1 << std::endl;

    images_[idx + 1] =
        vxCreateImage(context, width + 2 * disparity_padding_, height, format);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[idx + 1]));

    if (disparity_padding_ == 0) {
      images_[idx] = images_[idx + 1];
    } else {
      vx_rectangle_t roi;
      roi.start_x = disparity_padding_;
      roi.end_x = disparity_padding_ + width;
      roi.start_y = 0;
      roi.end_y = height;

      images_[idx] = vxCreateImageFromROI(images_[idx + 1], &roi);
      VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[idx]));
    }
  } else {
    // std::cout << "Building normal image at layer " << idx << std::endl;

    images_[idx] = vxCreateImage(context, width, height, format);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[idx]));
  }
}

vx_image VxGaussianImageScaler::addToGraph(vx_context context, vx_graph graph,
                                           vx_image input) {
  // This function returns the final stage of the scaler
  // (remembering that vw_image is a non-smart pointer and can be
  // referenced multiple times.
  //
  // This function needs to handle four different cases.
  //
  // Not downsampling (downsample_log2 == 0) AND Not padding:
  //     output_image_ = input
  //
  // Not downsampling (downsample_log2 == 0) AND Padding
  //     output_image_ = image the size of (input + padding)
  //     add a graph node which copies input into ROI of output_image_
  //
  // Downsampling and Not padding:
  //     Create a pyramid of ScaleGaussianNodes
  //     return output of lowest level
  //
  // Downsampling and Padding:
  //     Create a pyramid of ScaleGaussianNodes
  //     At lowest layer, output_image_ is (image + padding)
  //     Final ScaleGaussianNode writes into ROI of output_image_

  // Retrieve size and format of input image
  vx_uint32 input_width, input_height;
  VX_CHECK_STATUS(
      vxQueryImage(input, VX_IMAGE_WIDTH, &input_width, sizeof(vx_uint32)));
  VX_CHECK_STATUS(
      vxQueryImage(input, VX_IMAGE_HEIGHT, &input_height, sizeof(vx_uint32)));

  vx_df_image input_format;
  VX_CHECK_STATUS(
      vxQueryImage(input, VX_IMAGE_FORMAT, &input_format, sizeof(vx_df_image)));

  // This is the simplest case
  if ((downsample_log2_ == 0) && (disparity_padding_ == 0)) {
    return input;
  }

  images_.resize(downsample_log2_ + 2);

  if (downsample_log2_ == 0) {
    // With disparity padding, special case
    addImage(0, context, input_width, input_height, input_format);

    vx_node copy_node = nvxCopyImageNode(graph, input, images_[0]);
    VX_CHECK_STATUS(vxVerifyGraph(graph));
    vxReleaseNode(&copy_node);
  } else {
    uint32_t layer_width = input_width;
    uint32_t layer_height = input_height;

    const vx_int32 gaussian_kernel_size =
        3;  //\todo{} Do we care enough to make this configurable?

    images_[0] = input;

    for (int i = 1; i <= downsample_log2_; i++) {
      std::cout << "Building pyramid layer " << i << std::endl;

      layer_width = (layer_width + 1) / 2;
      layer_height = (layer_height + 1) / 2;

      addImage(i, context, layer_width, layer_height, input_format);

      vx_node scale_node = vxHalfScaleGaussianNode(
          graph, images_[i - 1], images_[i], gaussian_kernel_size);
      VX_CHECK_STATUS(vxGetStatus((vx_reference)scale_node));
      VX_CHECK_STATUS(vxVerifyGraph(graph));
      vxReleaseNode(&scale_node);
    }
  }

  //   else {
  //     // Downsampling

  //     std::cout << "Downsampling by " << downsample_log2_ << std::endl;

  //     // \todo{amarburg}  This doesn't work if downsample_log2_ = 1
  //     if (i == 0) {

  //     } else {
  //       if (i == (downsample_log2_ - 1)) {
  //         if (disparity_padding_ == 0) {
  //           images_[i] =
  //               vxCreateImage(context, layer_width, layer_height,
  //               input_format);
  //           VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[i]));

  //           output_image_ = images_[i];
  //           VX_CHECK_STATUS(vxGetStatus((vx_reference)output_image_));
  //           std::cout << "Check output_image_" << std::endl;
  //         } else {
  //           output_image_ =
  //               vxCreateImage(context, layer_width * 2 * disparity_padding_,
  //                             layer_height, input_format);
  //           VX_CHECK_STATUS(vxGetStatus((vx_reference)output_image_));
  //           std::cout << "Check output_image_" << std::endl;

  //           vx_rectangle_t roi;
  //           roi.start_x = disparity_padding_;
  //           roi.end_x = disparity_padding_ + layer_width;
  //           roi.start_y = 0;
  //           roi.end_y = layer_height;

  //           images_[i] = vxCreateImageFromROI(output_image_, &roi);
  //         }

  //       } else {
  //         images_[i] =
  //             vxCreateImage(context, layer_width, layer_height,
  //             input_format);
  //         VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[i]));
  //       }

  //       uint32_t p_width, p_height;
  //       uint32_t n_width, n_height;

  //       VX_CHECK_STATUS(vxQueryImage(images_[i - 1], VX_IMAGE_WIDTH,
  //       &p_width,
  //                                    sizeof(vx_uint32)));
  //       VX_CHECK_STATUS(vxQueryImage(images_[i - 1], VX_IMAGE_HEIGHT,
  //       &p_height,
  //                                    sizeof(vx_uint32)));
  //       VX_CHECK_STATUS(vxQueryImage(images_[i], VX_IMAGE_WIDTH, &n_width,
  //                                    sizeof(vx_uint32)));
  //       VX_CHECK_STATUS(vxQueryImage(images_[i], VX_IMAGE_HEIGHT, &n_height,
  //                                    sizeof(vx_uint32)));

  //       std::cout << "Scaling from " << p_width << " x " << p_height << "  to
  //       "
  //                 << n_width << " x " << n_height << std::endl;

  //       vx_node scale_node = vxHalfScaleGaussianNode(
  //           graph, images_[i - 1], images_[i], gaussian_kernel_size);
  //       VX_CHECK_STATUS(vxGetStatus((vx_reference)scale_node));

  //       VX_CHECK_STATUS(vxVerifyGraph(graph));
  //       vxReleaseNode(&scale_node);
  //     }
  //   }
  // }

  vx_image output_image = images_.back();
  VX_CHECK_STATUS(vxGetStatus((vx_reference)output_image));
  return output_image;
}

}  // namespace gpu_stereo_image_proc_visionworks
