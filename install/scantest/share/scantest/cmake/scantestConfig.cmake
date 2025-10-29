# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_scantest_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED scantest_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(scantest_FOUND FALSE)
  elseif(NOT scantest_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(scantest_FOUND FALSE)
  endif()
  return()
endif()
set(_scantest_CONFIG_INCLUDED TRUE)

# output package information
if(NOT scantest_FIND_QUIETLY)
  message(STATUS "Found scantest: 0.0.0 (${scantest_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'scantest' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${scantest_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(scantest_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${scantest_DIR}/${_extra}")
endforeach()
