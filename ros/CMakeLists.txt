cmake_minimum_required(VERSION 3.24)
project(kiss_matcher_ros)

# Set a default build type if none is specified
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    # Provide options for cmake-gui or ccmake
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_policy(SET CMP0074 NEW)

option(USE_ADDRESS_SANITIZER "Use address sanitizer" OFF)
if (USE_ADDRESS_SANITIZER)
    add_compile_options(-fsanitize=address -fsanitize-recover=address)
    add_link_options(-fsanitize=address -fsanitize-recover=address)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_matcher/ AND NOT IS_SYMLINK ${CMAKE_CURRENT_SOURCE_DIR})
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_matcher ${CMAKE_CURRENT_BINARY_DIR}/kiss_matcher)
else()
  #This is what kiss_icp does.
    #cmake_minimum_required(VERSION 3.18)
    #message(STATUS "Performing out-of-tree build, fetching KISS-Matcher v${CMAKE_PROJECT_VERSION} Release from Github")
    #include(FetchContent)
    #FetchContent_Declare(
    #  ext_kiss_matcher_core PREFIX kiss_matcher_core
    #  URL https://github.com/PRBonn/kiss-matcher/archive/refs/tags/v${CMAKE_PROJECT_VERSION}.tar.gz SOURCE_SUBDIR
    #      cpp/kiss_matcher)
    #FetchContent_MakeAvailable(ext_kiss_matcher_core)
endif()

# Load small_gicp
include(FetchContent)
FetchContent_Declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp
  GIT_TAG master
)
FetchContent_MakeAvailable(small_gicp)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(PCL REQUIRED)
include_directories(
    include
    src
    ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(GTSAM REQUIRED)

################################################################################
add_executable(run_kiss_matcher src/run_kiss_matcher.cpp)
target_include_directories(run_kiss_matcher
    PUBLIC
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(run_kiss_matcher
    kiss_matcher::kiss_matcher_core
    ${PCL_LIBRARIES}
)

add_executable(registration_visualizer src/registration_visualizer.cpp)
ament_target_dependencies(registration_visualizer
  rclcpp
  tf2_ros
  sensor_msgs
  pcl_conversions
  PCL
)
target_link_libraries(registration_visualizer
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
)

add_executable(kiss_matcher_sam src/slam/pose_graph_manager.cpp
                                src/slam/loop_closure.cpp
                                src/slam/loop_detector.cpp)
ament_target_dependencies(kiss_matcher_sam
  rclcpp
  tf2_ros
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  PCL
)
target_link_libraries(kiss_matcher_sam
  kiss_matcher::kiss_matcher_core
  ${PCL_LIBRARIES}
  small_gicp
  gtsam
)

install(TARGETS
  registration_visualizer
  run_kiss_matcher
  kiss_matcher_sam
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
