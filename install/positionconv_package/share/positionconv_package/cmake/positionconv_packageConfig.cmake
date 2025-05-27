# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_positionconv_package_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED positionconv_package_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(positionconv_package_FOUND FALSE)
  elseif(NOT positionconv_package_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(positionconv_package_FOUND FALSE)
  endif()
  return()
endif()
set(_positionconv_package_CONFIG_INCLUDED TRUE)

# output package information
if(NOT positionconv_package_FIND_QUIETLY)
  message(STATUS "Found positionconv_package: 0.0.0 (${positionconv_package_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'positionconv_package' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${positionconv_package_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(positionconv_package_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${positionconv_package_DIR}/${_extra}")
endforeach()
