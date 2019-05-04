cmake_minimum_required(VERSION ${MIN_CMAKE_VERSION})
project(sensor_fusion)

set(SENSOR_FUSION_SRCS
  src/States.cpp
  src/AncLocalization.cpp
  src/Initializer.cpp
  src/Estimator.cpp
  src/Robot.cpp
  src/CLSystem.cpp
)

add_library(${PROJECT_NAME} ${SENSOR_FUSION_SRCS})

target_include_directories(${PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
  PUBLIC ${eigen3_SOURCE_DIR}
  PUBLIC ${glog_SOURCE_DIR} ${glog_BINARY_DIR})

target_link_libraries(${PROJECT_NAME}
  PUBLIC glog)

# Build tests
if (BUILD_TESTS) 
  enable_testing()
  set(TEST_PROJECT_NAME ${PROJECT_NAME}_test)
  add_executable(${TEST_PROJECT_NAME}
    test/test_main.cpp
    test/States_test.cpp
    test/AncLocalization_test.cpp
    test/EnergyDetector_test.cpp)
  target_link_libraries(${TEST_PROJECT_NAME}
    gtest
    ${PROJECT_NAME})
  add_test(NAME ${TEST_PROJECT_NAME} COMMAND ${TEST_PROJECT_NAME})
endif()
