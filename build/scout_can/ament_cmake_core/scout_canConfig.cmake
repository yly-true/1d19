# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_scout_can_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED scout_can_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(scout_can_FOUND FALSE)
  elseif(NOT scout_can_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(scout_can_FOUND FALSE)
  endif()
  return()
endif()
set(_scout_can_CONFIG_INCLUDED TRUE)

# output package information
if(NOT scout_can_FIND_QUIETLY)
  message(STATUS "Found scout_can: 0.1.0 (${scout_can_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'scout_can' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT scout_can_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(scout_can_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${scout_can_DIR}/${_extra}")
endforeach()
