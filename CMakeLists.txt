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

# Option for build configuration
option (BUILD_TESTS "Builds all gtests" ON) 

# Download Eigen 3.3.6 
include(${CMAKE_SOURCE_DIR}/cmake/DownloadProject.cmake)

download_project(PROJ               eigen3
                HG_REPOSITORY       https://bitbucket.org/eigen/eigen/
                HG_TAG              3.3.6
                PREFIX              ${CMAKE_BINARY_DIR}/third_party/eigen3
                UPDATE_DISCONNECTED 1
)

# Build local dependencies
add_subdirectory(sensor_fusion)

add_subdirectory(io)
add_dependencies(io sensor_fusion)

# Build the app
add_executable(${PROJECT_NAME}_app apps/src/main.cpp)
target_include_directories(${PROJECT_NAME}_app PRIVATE
  io/include
  sensor_fusion/include)
# Link libraties
target_link_libraries(${PROJECT_NAME}_app
  io
  sensor_fusion)

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