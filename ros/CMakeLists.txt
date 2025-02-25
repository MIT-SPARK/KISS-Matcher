cmake_minimum_required(VERSION 3.8)
project(pointcloud_visualizer)

cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(robin REQUIRED)
find_package(kiss_matcher REQUIRED)

add_executable(pointcloud_visualizer src/pointcloud_visualizer.cpp)
ament_target_dependencies(pointcloud_visualizer
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  Eigen3
  teaserpp
  robin
  kiss_matcher
)
target_link_libraries(pointcloud_visualizer
  Eigen3::Eigen
  teaserpp::teaser_registration
  teaserpp::teaser_io
  TBB::tbb
  kiss_matcher::kiss_matcher_core
  robin::robin
  ${PCL_LIBRARIES}
)

install(TARGETS
  pointcloud_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
