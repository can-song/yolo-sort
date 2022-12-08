cmake_minimum_required(VERSION 3.3)

project(yolo-sort)

# set(CMAKE_CXX_STANTARD=11)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# message(STATUS ====${EIGEN3_INCLUDE_DIR}====)
include_directories("/usr/local/include/eigen3/")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(kalman src/kalman.cpp)

add_executable(test_eigen ${CMAKE_CURRENT_SOURCE_DIR}/example/test_eigen.cpp)
add_executable(test_kalman ${CMAKE_CURRENT_SOURCE_DIR}/example/test_kalman.cpp)
target_link_libraries(test_kalman kalman)