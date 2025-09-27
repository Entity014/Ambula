# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ambula_safety_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ambula_safety_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ambula_safety_FOUND FALSE)
  elseif(NOT ambula_safety_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ambula_safety_FOUND FALSE)
  endif()
  return()
endif()
set(_ambula_safety_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ambula_safety_FIND_QUIETLY)
  message(STATUS "Found ambula_safety: 0.0.0 (${ambula_safety_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ambula_safety' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ambula_safety_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ambula_safety_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ambula_safety_DIR}/${_extra}")
endforeach()
