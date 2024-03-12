# gpu_stereo_image_proc

> NOTE:  This repo is focused on the [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/).  The primary branch is `jetpack-4.5.1` to match the current distribution on the Xavier-NX-based Trisect.   It's strongly tied to the Visionworks and VPI APIs installed by that version of Jetpack.

The `jetpack-5.1.2` is our current "next development" branch for that version of Jetpack on the new Orin-NX-based development system.

## Overview

This package provides the baseline stereo capabilities on our [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/).  Given that focus, our primary development environment is on the two generations of the Trisect:

* The original Trisect used a Jetson NX running Jetpack 4.4.x. assuming the Trisect's customized [OpenCV and ROS1](https://gitlab.com/rsa-perception-sensor/trisect_environment) build in branch `trisect-dev`.   The version prioritizes `gpu_stereo_image_visionworks` as the version `VPI 0.3` included in that release of Jetpack is fairly immature.
* The next-generation Trisect uses an Orin NX with Jetpack 5.1.2.  This version prioritizes `gpu_stereo_image_proc_vpi` as it provides the more-mature `VPI 2.1` while Visionworks is depcrecated.

As the two version of VPI are API-incompatible, for now the two branches are maintained separately.

This repo includes multiple GPU-accelerated Semi-Global Block Matching (SGBM) algorithms:

* [NVIDIA VisionWorks](https://developer.nvidia.com/embedded/visionworks).  Per above, this is the preferred version on the current Trisect.   Expect to be phased out as Visionworks is deprectaed in later version of Jetpack.
* NVIDIA VPI.  This is the preferred version on the next generation of Trisect.
* [Fixstars libSGM](https://github.com/fixstars/libSGM), however this ROS package is disabled by default.  It can be enabled by setting the CMake variable `BUILD_GPU_STEREO_IMAGE_PROC_LIBSGM`:

```
catkin config --cmake-args -DBUILD_GPU_STEREO_IMAGE_PROC_LIBSGM=True
```

* A version using the CUDA-accelerated algorithms in OpenCV is in progress

Each algorithm is in its own ROS package, compilation of individual packages can be disabled by adding them to the [Catkin skiplist](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html#buildlisting-and-skiplisting-packages):

```
catkin config --skiplist gpu_stereo_image_proc_opencv
```


## Licenses

This version is based on [whill-lab's](https://github.com/whill-labs) [upstream package](https://github.com/whill-labs/gpu_stereo_image_proc), but has diverged significantly in structure.

This software maintain's the upstream package's [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

The underlying code of this package is forked from [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc) which is distibuted under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

This package includes [libSGM](https://github.com/fixstars/libSGM) which is distributed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0).

This package depends on [VisionWorks](https://developer.nvidia.com/embedded/visionworks) which NVIDIA reserves all the copyrights of.
