cmake_minimum_required(VERSION 3.16...3.31)
project(kiss_matcher VERSION 0.3.0 LANGUAGES CXX)

message("")
message("██╗  ██╗██╗███████╗███████╗      ")
message("██║ ██╔╝██║██╔════╝██╔════╝      ")
message("█████╔╝ ██║███████╗███████╗█████╗")
message("██╔═██╗ ██║╚════██║╚════██║╚════╝")
message("██║  ██╗██║███████║███████║      ")
message("╚═╝  ╚═╝╚═╝╚══════╝╚══════╝      ")
message("")
message("███╗   ███╗ █████╗ ████████╗ ██████╗██╗  ██╗███████╗██████╗ ")
message("████╗ ████║██╔══██╗╚══██╔══╝██╔════╝██║  ██║██╔════╝██╔══██╗")
message("██╔████╔██║███████║   ██║   ██║     ███████║█████╗  ██████╔╝")
message("██║╚██╔╝██║██╔══██║   ██║   ██║     ██╔══██║██╔══╝  ██╔══██╗")
message("██║ ╚═╝ ██║██║  ██║   ██║   ╚██████╗██║  ██║███████╗██║  ██║")
message("╚═╝     ╚═╝╚═╝  ╚═╝   ╚═╝    ╚═════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝")
message("")

option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" OFF)
option(USE_SYSTEM_ROBIN "Use system pre-installed ROBIN from SPARK @ MIT" ON)
option(USE_ADDRESS_SANITIZER "Use address sanitizer" OFF)

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

if (USE_ADDRESS_SANITIZER)
    add_compile_options(-fsanitize=address -fsanitize-recover=address)
    add_link_options(-fsanitize=address -fsanitize-recover=address)
endif()

include(GNUInstallDirs)
include(3rdparty/find_dependencies.cmake)
include(cmake/CompilerOptions.cmake)

# NOTE(hlim): Without this line, pybinded KISS-Matcher does not work
find_library(LZ4_LIBRARY lz4 REQUIRED)

include(FindOpenMP) #The best way to set proper compiler settings for using OpenMP in all platforms
if (OPENMP_FOUND) #The best way to set proper compiler settings for using OpenMP in all platforms
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else (OPENMP_FOUND)
    message("ERROR: OpenMP could not be found.")
endif (OPENMP_FOUND)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/core)

set(TARGET_NAME kiss_matcher_core)
# NOTE(hlim): Without `STATIC`, it leads to ImportError in Pybinding as follows:
# "libkiss_matcher_core.so: cannot open shared object file: NO such file or directory ..."
# See Issue #37 (https://github.com/MIT-SPARK/KISS-Matcher/issues/37)
add_library(${TARGET_NAME} STATIC
    core/kiss_matcher/ROBINMatching.cpp
    core/kiss_matcher/FasterPFH.cpp
    core/kiss_matcher/KISSMatcher.cpp
    core/kiss_matcher/GncSolver.cpp
)
add_library(kiss_matcher::kiss_matcher_core ALIAS kiss_matcher_core)

target_link_libraries(${TARGET_NAME}
    PUBLIC Eigen3::Eigen robin::robin ${OpenMP_LIBS} ${EIGEN3_LIBS} TBB::tbb ${LZ4_LIBRARY}
)

# To make kiss_matcher::core global for Pybinding
set_global_target_properties(${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS ${TARGET_NAME}
    EXPORT ${PROJECT_NAME}-export
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT ${PROJECT_NAME}-export
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
)

install(DIRECTORY core/kiss_matcher DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

export(TARGETS ${MYPROJECT_EXPORTED_TARGETS} FILE ${PROJECT_NAME}-exports.cmake)

install(FILES cmake/${PROJECT_NAME}Config.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
