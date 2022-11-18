cmake_minimum_required(VERSION 3.5)
project(gpu_stereo_image_proc_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_proc REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(cv_bridge dynamic_reconfigure image_geometry
#         image_proc image_transport message_filters nodelet
#         sensor_msgs stereo_msgs)

# find_package(cv_bridge
#         image_proc image_transport message_filters nodelet
#         sensor_msgs stereo_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  #add_compile_options(-std=c++14 )

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Dynamic reconfigure support
# generate_dynamic_reconfigure_options(
#   cfg/DisparityBilateralFilter.cfg
#   cfg/DisparityWLSFilter.cfg
#   )

# catkin_package(
#   CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
# )

include_directories(include)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${image_proc_INCLUDE_DIRS})

# find_package(CUDA REQUIRED)
# set(CUDA_ARCH "-arch=sm_72" CACHE STRING "Value of the NVCC -arch option.")
# if(${CMAKE_VERSION} VERSION_EQUAL 3.7 OR ${CMAKE_VERSION} VERSION_GREATER 3.7)
#   option(AUTO_DETECT_ARCH "Detect local GPU compute arch automatically" ON)
# endif()

# if(DEFINED AUTO_DETECT_ARCH AND "${AUTO_DETECT_ARCH}")
# 	CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS "Auto")
# 	set(CUDA_ARCH "${ARCH_FLAGS}" CACHE STRING "Value of the NVCC -arch option." FORCE)
# endif()

# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

add_library(${PROJECT_NAME}
            src/sgbm_processor.cpp
            src/msg_conversions.cpp
            src/camera_info_conversions.cpp )

ament_target_dependencies(${PROJECT_NAME} image_proc sensor_msgs)

target_link_libraries(${PROJECT_NAME}
                                      ${OpenCV_LIBRARIES} )

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

# install the include directory
install(DIRECTORY include/
        DESTINATION include
        #${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

ament_export_include_directories(include) #Apparently this is needed for inporting of includes in other packages

ament_package()

