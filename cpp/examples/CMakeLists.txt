cmake_minimum_required(VERSION 3.10)
project(kiss_matcher_example)

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

include(FetchContent)

find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(PCL REQUIRED)
### To include ROBIN& KISS-Matcher ###
find_package(robin REQUIRED)
find_package(kiss_matcher REQUIRED)

# The best way to set proper compiler settings for using OpenMP in all platforms
include(FindOpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else (OPENMP_FOUND)
    message("ERROR: OpenMP could not be found.")
endif (OPENMP_FOUND)

option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
include(3rdparty/find_dependencies.cmake)
include(3rdparty/download_datasets.cmake)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    include
    src
)

add_executable(run_matching_using_bunny src/run_matching_using_bunny.cc)
target_include_directories(run_matching_using_bunny
    PUBLIC
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(run_matching_using_bunny
    Eigen3::Eigen
    teaserpp::teaser_registration
    teaserpp::teaser_io
    TBB::tbb
    kiss_matcher::kiss_matcher_core
    robin::robin
    ${PCL_LIBRARIES}
)

add_executable(run_kiss_matcher src/run_kiss_matcher.cc)
target_include_directories(run_kiss_matcher
    PUBLIC
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(run_kiss_matcher
    Eigen3::Eigen
    TBB::tbb
    kiss_matcher::kiss_matcher_core
    robin::robin
    ${PCL_LIBRARIES}
)

add_executable(downsampling_speed_comparison src/downsampling_speed_comparison.cc)
target_link_libraries(downsampling_speed_comparison PRIVATE ${PCL_LIBRARIES} TBB::tbb)

add_executable(conversion_speed_comparison src/conversion_speed_comparison.cc)
target_link_libraries(conversion_speed_comparison PRIVATE ${PCL_LIBRARIES} TBB::tbb)
