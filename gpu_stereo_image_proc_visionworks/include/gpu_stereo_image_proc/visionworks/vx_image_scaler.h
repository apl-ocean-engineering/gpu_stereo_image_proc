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
#pragma once

#include <vector>

// For cv::Size
#include <NVX/nvx.h>
#include <VX/vx.h>
#include <VX/vxu.h>

#include <opencv2/core.hpp>

namespace gpu_stereo_image_proc_visionworks {

class VxImageScaler {
 public:
  VxImageScaler(unsigned int downsample_log2, unsigned int disparity_padding);

  virtual vx_image addToGraph(vx_context context, vx_graph graph,
                              vx_image input) = 0;

  // int downsample() const { return 2 ^ downsample_log2_; }

  // Note outputSize() is not set until addToGraph is called
  // const cv::Size outputSize() const { return output_size_; }

 protected:
  // cv::Size output_size_;
  // vx_image output_image_;

  unsigned int downsample_log2_, disparity_padding_;

  // No default constructor
  VxImageScaler() = delete;
  // The class is non-copyable
  VxImageScaler(const VxImageScaler &) = delete;
  VxImageScaler &operator=(const VxImageScaler &) = delete;
};

class VxGaussianImageScaler : public VxImageScaler {
 public:
  VxGaussianImageScaler(unsigned int downsample_log2,
                        unsigned int disparity_padding);

  vx_image addToGraph(vx_context context, vx_graph graph,
                      vx_image input) override;

 protected:
  // This class downsamples using a Visionworks call which halves image size,
  // so we maintain a pyramid of intermediate images
  std::vector<vx_image> images_;

  void addImage(const size_t idx, vx_context context, const cv::Size &sz,
                vx_df_image format);

  // No default constructor
  VxGaussianImageScaler() = delete;
  // The class is non-copyable
  VxGaussianImageScaler(const VxGaussianImageScaler &) = delete;
  VxGaussianImageScaler &operator=(const VxGaussianImageScaler &) = delete;
};

}  // namespace gpu_stereo_image_proc_visionworks
