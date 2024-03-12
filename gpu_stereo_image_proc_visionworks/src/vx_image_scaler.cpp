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
                                     const cv::Size &sz, vx_df_image format) {
  assert((idx + 1) < images_.size());

  // Create next layer
  if (idx == downsample_log2_) {
    // std::cout << "Building output at layer " << idx+1 << std::endl;

    images_[idx + 1] = vxCreateImage(context, sz.width + 2 * disparity_padding_,
                                     sz.height, format);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[idx + 1]));

    // This works even if disparity_padding_==0
    // Creates a trivial full-sized ROI
    vx_rectangle_t roi;
    roi.start_x = disparity_padding_;
    roi.end_x = disparity_padding_ + sz.width;
    roi.start_y = 0;
    roi.end_y = sz.height;

    images_[idx] = vxCreateImageFromROI(images_[idx + 1], &roi);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[idx]));
  } else {
    // std::cout << "Building normal image at layer " << idx << std::endl;

    images_[idx] = vxCreateImage(context, sz.width, sz.height, format);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[idx]));
  }
}

vx_image VxGaussianImageScaler::addToGraph(vx_context context, vx_graph graph,
                                           vx_image input) {
  //
  // If downsample_log2 == 0 and disparity_padding == 0, then this function
  // is a straight passthrough:  return input
  //
  // Otherwise, it builds an vector<vx_image> with downsample_log2+2 layers
  // Layer[0] is the input image.
  //
  //    Layer[N] is the disparity-padded output
  //    Layer[N-1] is the appropriately scaled ROI into Layer[N]
  //
  // (Note that the above works fine even if disparity_padding = 0, you
  // get a trivial full-sized ROI into Layer[N-1]

  // If **not ** doing disparity padding
  //    Layer[N-1] = Layer[N] is the appropriately scaled image

  // So for downsample_log2 = 2 with a 1920x1200 input image:
  //
  // Without disparity padding:
  // Layer[0] = input image (1920 x 1200)
  // Layer[1] = scaled image (960 x 600)
  // Layer[2] = (480 x 300) ROI into Layer[3]
  // Layer[3] = Disparity padded image ( 480 x 380) <-- returned from function
  //
  // With disparity padding (disparity = 32)
  //   ...
  // Layer[2] = (480 x 300) ROI into Layer[3]
  // Layer[3] = Disparity padded image ( 544 (= 480 + 2*32) x 380) <-- returned
  // from function

  // Retrieve size and format of input image
  cv::Size input_sz;
  VX_CHECK_STATUS(
      vxQueryImage(input, VX_IMAGE_WIDTH, &input_sz.width, sizeof(vx_uint32)));
  VX_CHECK_STATUS(vxQueryImage(input, VX_IMAGE_HEIGHT, &input_sz.height,
                               sizeof(vx_uint32)));

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
    addImage(0, context, input_sz, input_format);

    vx_node copy_node = nvxCopyImageNode(graph, input, images_[0]);
    VX_CHECK_STATUS(vxVerifyGraph(graph));
    vxReleaseNode(&copy_node);
  } else {
    cv::Size layer_sz = input_sz;
    const vx_int32 gaussian_kernel_size =
        3;  //\todo{} Do we care enough to make this configurable?

    images_[0] = input;

    for (int i = 1; i <= downsample_log2_; i++) {
      // std::cout << "Building pyramid layer " << i << std::endl;

      // This is the output size of the vxHalfScaleGaussianNode node per
      // documentation
      layer_sz.width = (layer_sz.width + 1) / 2;
      layer_sz.height = (layer_sz.height + 1) / 2;

      addImage(i, context, layer_sz, input_format);

      vx_node scale_node = vxHalfScaleGaussianNode(
          graph, images_[i - 1], images_[i], gaussian_kernel_size);
      VX_CHECK_STATUS(vxGetStatus((vx_reference)scale_node));
      VX_CHECK_STATUS(vxVerifyGraph(graph));
      vxReleaseNode(&scale_node);
    }
  }

  vx_image output_image = images_.back();
  VX_CHECK_STATUS(vxGetStatus((vx_reference)output_image));
  return output_image;
}

}  // namespace gpu_stereo_image_proc_visionworks
