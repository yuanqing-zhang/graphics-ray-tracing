cmake_minimum_required(VERSION 3.0)

project(RayTracing)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenMP REQUIRED)
# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS "Eigen3 path is ${EIGEN3_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV path is ${OpenCV_INCLUDE_DIRS}")

add_subdirectory(src)