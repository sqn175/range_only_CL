cmake_minimum_required(VERSION 2.8.2)
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

# Build local dependencies
include_directories(io/include)
add_subdirectory(io)

# Target
add_executable(${PROJECT_NAME} main.cpp)

# Link libraties
target_link_libraries(${PROJECT_NAME} 
  io)

if (BUILD_TESTS)
  if (CMAKE_VERSION VERSION_LESS 3.2)
      set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
  else()
      set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
  endif()

  message(${CMAKE_CURRENT_LIST_DIR}, TEST)

  include(${CMAKE_SOURCE_DIR}/cmake/DownloadProject.cmake)
  download_project(PROJ               googletest
                  GIT_REPOSITORY      https://github.com/google/googletest.git
                  GIT_TAG             master
                  PREFIX              ${CMAKE_BINARY_DIR}/third_party/gtest
                  ${UPDATE_DISCONNECTED_IF_AVAILABLE}
  )
  # Prevent GoogleTest from overriding our compiler/linker options
  # when building with Visual Studio
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

  # When using CMake 2.8.11 or later, header path dependencies
  # are automatically added to the gtest and gmock targets.
  # For earlier CMake versions, we have to explicitly add the
  # required directories to the header search path ourselves.
  if (CMAKE_VERSION VERSION_LESS 2.8.11)
      include_directories("${gtest_SOURCE_DIR}/include"
                          "${gmock_SOURCE_DIR}/include")
  endif()
endif()


