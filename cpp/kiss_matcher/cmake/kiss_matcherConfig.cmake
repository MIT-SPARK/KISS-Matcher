get_filename_component(MYPROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
list(PREPEND CMAKE_MODULE_PATH "${MYPROJECT_CMAKE_DIR}")
include(CMakeFindDependencyMacro)

find_dependency(Eigen3 3.3 REQUIRED)
find_dependency(OpenMP REQUIRED)
# Use Findflann.cmake (installed alongside this file) which tries CONFIG mode
# first and falls back to manual discovery on distros without a flann CMake
# config (e.g. Ubuntu 22.04 libflann-dev).
find_dependency(flann REQUIRED)

# The parameter must adhere to the format: ${PROJECT_NAME}Targets.cmake
include("${MYPROJECT_CMAKE_DIR}/kiss_matcherTargets.cmake")
