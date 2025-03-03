cmake_minimum_required(VERSION 3.8)
project(kiss_matcher_ros)

cmake_policy(SET CMP0074 NEW)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(registration_visualizer src/registration_visualizer.cpp)
ament_target_dependencies(registration_visualizer
  rclcpp
  tf2_ros
  sensor_msgs
  pcl_conversions
  PCL
  Eigen3
)
target_link_libraries(registration_visualizer
  Eigen3::Eigen
  TBB::tbb
  kiss_matcher::kiss_matcher_core
  robin::robin
  ${PCL_LIBRARIES}
)

install(TARGETS
  registration_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
