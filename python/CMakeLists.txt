cmake_minimum_required(VERSION 3.16...3.26)

project(kiss_matcher_pybind)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message(STATUS "Python Interpreter Version: ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11
  GIT_TAG master
)
FetchContent_MakeAvailable(pybind11)

set(PYBIND11_NEWPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
# find_package(pybind11 CONFIG REQUIRED)

if (DEFINED SKBUILD)
    message(STATUS "Building with Scikit-Build")
endif ()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_matcher/)
  message(STATUS "Install KISS-Matcher using local C++ files...")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_matcher ${CMAKE_CURRENT_BINARY_DIR}/kiss_matcher)
else()
  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Performing out-of-tree build, fetching KISS-Matcher v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  FetchContent_Declare(
    ext_kiss_matcher_core PREFIX kiss_matcher
    URL https://github.com/LimHyungTae/kiss-matcher/archive/refs/tags/v0.3.1.tar.gz SOURCE_SUBDIR
        cpp/kiss_matcher)
  FetchContent_MakeAvailable(ext_kiss_matcher_core)
endif()

set(OUTPUT_MODULE_NAME kiss_matcher)

pybind11_add_module(${OUTPUT_MODULE_NAME} kiss_matcher/pybind/kiss_matcher_pybind.cpp)
target_link_libraries(${OUTPUT_MODULE_NAME} PUBLIC kiss_matcher_core)

# To ignore warning messages
target_compile_options(${OUTPUT_MODULE_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-w>
    $<$<CXX_COMPILER_ID:Clang>:-w>
    $<$<CXX_COMPILER_ID:MSVC>:/w>
)

# fix for clang
# see: https://github.com/pybind/pybind11/issues/1818
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(${OUTPUT_MODULE_NAME} PUBLIC -fsized-deallocation)
endif ()

install(TARGETS ${OUTPUT_MODULE_NAME} DESTINATION .)



# add_subdirectory(kiss_matcher/pybind)
