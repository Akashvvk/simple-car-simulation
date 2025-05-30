# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_simple_car_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED simple_car_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(simple_car_FOUND FALSE)
  elseif(NOT simple_car_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(simple_car_FOUND FALSE)
  endif()
  return()
endif()
set(_simple_car_CONFIG_INCLUDED TRUE)

# output package information
if(NOT simple_car_FIND_QUIETLY)
  message(STATUS "Found simple_car: 0.0.0 (${simple_car_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'simple_car' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${simple_car_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(simple_car_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${simple_car_DIR}/${_extra}")
endforeach()
