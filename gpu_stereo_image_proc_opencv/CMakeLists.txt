cmake_minimum_required(VERSION 3.1)
project(gpu_stereo_image_proc_opencv)

find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_geometry image_proc
            image_transport message_filters nodelet sensor_msgs stereo_msgs
            gpu_stereo_image_proc_common)
find_package(Boost REQUIRED COMPONENTS thread)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  #add_compile_options(-std=c++14 )

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/OpenCVSGBM.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

find_package(OpenCV 4 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

find_package(CUDA REQUIRED)
set(CUDA_ARCH "-arch=sm_72" CACHE STRING "Value of the NVCC -arch option.")
if(${CMAKE_VERSION} VERSION_EQUAL 3.7 OR ${CMAKE_VERSION} VERSION_GREATER 3.7)
  option(AUTO_DETECT_ARCH "Detect local GPU compute arch automatically" ON)
endif()

if(DEFINED AUTO_DETECT_ARCH AND "${AUTO_DETECT_ARCH}")
	CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS "Auto")
	set(CUDA_ARCH "${ARCH_FLAGS}" CACHE STRING "Value of the NVCC -arch option." FORCE)
endif()

# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Nodelet library
set(OPENCV_NODELET_NAME opencv_stereo_image_proc)
add_library(${OPENCV_NODELET_NAME}
#            src/libgpu_stereo_image_proc/vx_stereo_matcher_base.cpp
#            src/libgpu_stereo_image_proc/vx_stereo_matcher.cpp
#            src/libgpu_stereo_image_proc/vx_bidirectional_stereo_matcher.cpp
#            src/libgpu_stereo_image_proc/vx_conversions.cpp
#            src/libgpu_stereo_image_proc/msg_conversions.cpp
#            src/libgpu_stereo_image_proc/camera_info_conversions.cpp
            src/nodelet/opencv_disparity.cpp)
target_link_libraries(${OPENCV_NODELET_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES})

add_dependencies(${OPENCV_NODELET_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${OPENCV_NODELET_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
