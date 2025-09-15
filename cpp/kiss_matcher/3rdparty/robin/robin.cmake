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
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# NOTE(hlim) `OFF` means that we gonna generate static library to make it more independent
# Thus, `libpmc.a` will be created
option(PMC_BUILD_SHARED "Build pmc as a shared library (.so)" OFF)

include(FetchContent)
FetchContent_Declare(robin URL https://github.com/MIT-SPARK/ROBIN/archive/refs/tags/v.1.2.4.tar.gz)
FetchContent_GetProperties(robin)

# Prefer FetchContent_MakeAvailable when available (CMake 3.14+)
if(COMMAND FetchContent_MakeAvailable)
  FetchContent_MakeAvailable(robin)

  # Mark ROBIN's include dirs as 'SYSTEM' to ignore third-party warnings
  get_target_property(_robin_inc robin INTERFACE_INCLUDE_DIRECTORIES)
  if(_robin_inc)
    set_target_properties(robin PROPERTIES
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_robin_inc}")
  endif()

# Fallback for older/special environments: Populate + add_subdirectory
else()
  # On CMake 3.30+, suppress CMP0169 warning for FetchContent_Populate by setting it to OLD
  if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
  endif()

  FetchContent_GetProperties(robin)
  if(NOT robin_POPULATED)
    FetchContent_Populate(robin)
    # Before 3.25 there is no SYSTEM option, so set system includes manually
    add_subdirectory(${robin_SOURCE_DIR} ${robin_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(_robin_inc robin INTERFACE_INCLUDE_DIRECTORIES)
    if(_robin_inc)
      set_target_properties(robin PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_robin_inc}")
    endif()
  endif()
endif()
