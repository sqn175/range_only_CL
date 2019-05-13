set(MIN_CMAKE_VERSION 2.8.11)
cmake_minimum_required(VERSION ${MIN_CMAKE_VERSION})
project(rcl)

# The version number
set(RCL_MAJOR_VERSION 1)
set(RCL_MINOR_VERSION 1)
set(RCL_PATCH_VERSION 1)
set(RCL_VERSION
  ${RCL_MAJOR_VERSION}.${RCL_MINOR_VERSION}.${RCL_PATCH_VERSION})

set(CMAKE_CXX_STANDARD 11)
# TO debug some optimized code
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall  -O0 -march=native ")

# Option for build configuration
option (BUILD_APP "Builds the application" ON)
option (BUILD_TESTS "Builds all gtests" ON) 

# Plugin in download script to add externerl project
include(${CMAKE_SOURCE_DIR}/cmake/DownloadProject.cmake)

# Download google glog 0.4.0
download_project(PROJ                glog
                GIT_REPOSITORY       https://github.com/google/glog.git
                GIT_TAG              v0.4.0
                PREFIX              ${CMAKE_BINARY_DIR}/third_party/glog
                UPDATE_DISCONNECTED 1
)
# Build glog directly from the download subdirectory
add_subdirectory(${glog_SOURCE_DIR})

# Download Eigen 3.3.6 
download_project(PROJ               eigen3
                HG_REPOSITORY       https://bitbucket.org/eigen/eigen/
                HG_TAG              3.3.6
                PREFIX              ${CMAKE_BINARY_DIR}/third_party/eigen3
                UPDATE_DISCONNECTED 1
)

# Build local dependencies
add_subdirectory(sensor_fusion)
add_dependencies(sensor_fusion glog)

add_subdirectory(io)
add_dependencies(io sensor_fusion)

# Build the app
if (BUILD_APP)
  # Download Pangolin for results display
  download_project( PROJ                pangolin
                    GIT_REPOSITORY      https://github.com/stevenlovegrove/Pangolin.git
                    GIT_TAG             master
                    PREFIX              ${CMAKE_BINARY_DIR}/third_party/pangolin
                    UPDATE_DISCONNECTED 1
  )
  add_subdirectory(${pangolin_SOURCE_DIR} ${pangolin_BINARY_DIR})

  add_executable(${PROJECT_NAME}_sync_app apps/src/sync_app_main.cpp)
  target_include_directories(${PROJECT_NAME}_sync_app PUBLIC
    io/include
    sensor_fusion/include
    ${pangolin_SOURCE_DIR}/include
  )
  # Link libraties
  target_link_libraries(${PROJECT_NAME}_sync_app
    io
    sensor_fusion
    pangolin
    )
endif()

if (BUILD_TESTS)
  download_project(PROJ               googletest
                  GIT_REPOSITORY      https://github.com/google/googletest.git
                  GIT_TAG             master
                  PREFIX              ${CMAKE_BINARY_DIR}/third_party/gtest
                  UPDATE_DISCONNECTED 1
  )
  # Prevent GoogleTest from overriding our compiler/linker options
  # when building with Visual Studio
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()