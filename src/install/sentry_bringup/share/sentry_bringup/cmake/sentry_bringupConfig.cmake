# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sentry_bringup_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sentry_bringup_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sentry_bringup_FOUND FALSE)
  elseif(NOT sentry_bringup_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sentry_bringup_FOUND FALSE)
  endif()
  return()
endif()
set(_sentry_bringup_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sentry_bringup_FIND_QUIETLY)
  message(STATUS "Found sentry_bringup: 0.0.0 (${sentry_bringup_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sentry_bringup' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sentry_bringup_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sentry_bringup_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sentry_bringup_DIR}/${_extra}")
endforeach()
