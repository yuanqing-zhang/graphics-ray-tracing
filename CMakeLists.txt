cmake_minimum_required(VERSION 3.0)

project(RayTracing)

set( CMAKE_CXX_FLAGS "-std=c++11" )

# Eigen3
find_package(Eigen3 3.3.7 REQUIRED)
message(STATUS "Eigen3 path is ${EIGEN3_INCLUDE_DIRS}")
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)
message(STATUS "OpenMP path is ${OPENMP_INCLUDE_DIRS}")

add_subdirectory(src)