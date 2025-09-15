# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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
option(BUILD_SHARED_LIBS "TBB BUILD_SHARED_LIBS" ON)
option(TBBMALLOC_BUILD  "TBB TBBMALLOC_BUILD" ON)
option(TBB_EXAMPLES "TBB TBB_EXAMPLES" OFF)
option(TBB_STRICT "TBB TBB_STRICT" OFF)
option(TBB_TEST "TBB TBB_TEST" OFF)
#option(TBB_SANITIZE "TBB TBB_SANITIZE" "thread")

include(FetchContent)
FetchContent_Declare(tbb URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2022.0.0.tar.gz)
FetchContent_GetProperties(tbb)
# Prefer FetchContent_MakeAvailable when available (CMake 3.14+)
if(COMMAND FetchContent_MakeAvailable)
  FetchContent_MakeAvailable(tbb)

  # Mark TBB's include dirs as 'SYSTEM' to ignore third-party warnings
  get_target_property(_tbb_inc tbb INTERFACE_INCLUDE_DIRECTORIES)
  if(_tbb_inc)
    set_target_properties(tbb PROPERTIES
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_tbb_inc}")
  endif()

# Fallback for older/special environments: Populate + add_subdirectory
else()
  # On CMake 3.30+, suppress CMP0169 warning for FetchContent_Populate by setting it to OLD
  if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
  endif()

  FetchContent_GetProperties(tbb)
  if(NOT tbb_POPULATED)
    FetchContent_Populate(tbb)
    # Before 3.25 there is no SYSTEM option, so set system includes manually
    add_subdirectory(${tbb_SOURCE_DIR} ${tbb_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(_tbb_inc tbb INTERFACE_INCLUDE_DIRECTORIES)
    if(_tbb_inc)
      set_target_properties(tbb PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_tbb_inc}")
    endif()
  endif()
endif()
