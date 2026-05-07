# MIT License
#
# Copyright (c) 2025 Hyungtae Lim and coauthors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

# Locate the flann library (https://github.com/flann-lib/flann) and expose
# `flann::flann_cpp` as an imported target.
#
# Why this module exists:
#   flann ships a CMake config (`flann-config.cmake`) on Homebrew (macOS),
#   Ubuntu 24.04+, vcpkg, and similar. But Ubuntu 22.04's `libflann-dev`
#   does NOT install a CMake config — it only places headers under
#   `/usr/include/flann` and libraries under `/usr/lib`. So plain
#   `find_package(flann CONFIG)` fails on Ubuntu 22.04.
#
# Strategy:
#   1) Prefer the upstream CMake config when available.
#   2) Fall back to manual header/library discovery and synthesize the
#      same `flann::flann_cpp` imported target.

if(TARGET flann::flann_cpp)
  set(flann_FOUND TRUE)
  return()
endif()

# 1) Prefer flann's upstream CMake config.
find_package(flann CONFIG QUIET)
if(flann_FOUND AND TARGET flann::flann_cpp)
  return()
endif()

# 2) Manual fallback (e.g. Ubuntu 22.04 libflann-dev).
find_path(FLANN_INCLUDE_DIR
  NAMES flann/flann.hpp
  DOC "flann include directory")
find_library(FLANN_CPP_LIBRARY
  NAMES flann_cpp
  DOC "flann_cpp library")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(flann
  REQUIRED_VARS FLANN_INCLUDE_DIR FLANN_CPP_LIBRARY)

if(flann_FOUND)
  add_library(flann::flann_cpp UNKNOWN IMPORTED)
  set_target_properties(flann::flann_cpp PROPERTIES
    IMPORTED_LOCATION "${FLANN_CPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${FLANN_INCLUDE_DIR}")
endif()

mark_as_advanced(FLANN_INCLUDE_DIR FLANN_CPP_LIBRARY)
