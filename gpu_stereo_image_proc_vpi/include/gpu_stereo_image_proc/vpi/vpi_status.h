#pragma once

#include <vpi/Status.h>
#include <vpi/Version.h>

#define VPI_CHECK_STATUS(s)                                    \
  do {                                                         \
    const auto __status = (s);                                 \
    if (__status != VPI_SUCCESS) {                             \
      ROS_ERROR("[%d] VPI ERROR (%d): %s", __LINE__, __status, \
                vpiStatusGetName(__status));                   \
    }                                                          \
    ROS_ASSERT(__status == VPI_SUCCESS);                       \
  } while (false)

#define VPI_VERSION_WITH_VIEWS NV_VPI_MAKE_VERSION(2, 0, 0)
