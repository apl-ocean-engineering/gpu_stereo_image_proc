# gpu_stereo_image_proc

> NOTE:  This repo is focused on the [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/).  This primary branch `trisect-5.1.4` is focused on the current development version based on [Jetpack 5.1.4](https://docs.nvidia.com/jetson/archives/jetpack-archived/jetpack-514/release-notes/index.html)and uses the [block matcher provided by VPI 2.4.](https://docs.nvidia.com/vpi/2.4/algo_stereo_disparity.html)

## Overview

This package provides the baseline stereo capabilities on our [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/).

The next-generation Trisect uses an Orin NX with Jetpack 5.1.4.  This version prioritizes `gpu_stereo_image_proc_vpi` as it provides the more-mature [VPI 2.4.](https://docs.nvidia.com/vpi/2.4/algo_stereo_disparity.html)

A version using the CUDA-accelerated algorithms in OpenCV is in progress

Each algorithm is in its own ROS package, compilation of individual packages can be disabled by adding them to the [Catkin skiplist](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html#buildlisting-and-skiplisting-packages):

```
catkin config --skiplist gpu_stereo_image_proc_opencv
```


## Licenses

This version is based on [whill-lab's](https://github.com/whill-labs) [upstream package](https://github.com/whill-labs/gpu_stereo_image_proc), but has diverged significantly in structure.

This software maintain's the upstream package's [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

The underlying code of this package is forked from [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc) which is distibuted under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).
