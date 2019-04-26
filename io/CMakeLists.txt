cmake_minimum_required(VERSION ${MIN_CMAKE_VERSION})
project(io)

# Target
set(IO_SRCS
    src/AsyncSerial.cpp
    src/DataProtocol.cpp)

# Link libraries
set(BOOST_LIBS system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} ${IO_SRCS})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  sensor_fusion
  glog)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
  PUBLIC ${glog_SOURCE_DIR} ${glog_BINARY_DIR})
  
# Build tests
if (BUILD_TESTS) 
  enable_testing()
  set(TEST_PROJECT_NAME ${PROJECT_NAME}_test)
  add_executable(${TEST_PROJECT_NAME}
    test/test_main.cpp
    test/DataProtocol_test.cpp)
  target_link_libraries(${TEST_PROJECT_NAME}
    gtest
    ${PROJECT_NAME})
  add_test(NAME ${TEST_PROJECT_NAME} COMMAND ${TEST_PROJECT_NAME})
endif()
