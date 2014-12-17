###
### Quentin Rouxel
### Thesis (2014-201?)
###

cmake_minimum_required(VERSION 2.8)
project(Thesis)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Include Eigen (linear algebra template library)
set(EIGEN_DIRECTORY "Eigen" CACHE STRING "Eigen lib directory")
include_directories(${EIGEN_DIRECTORY})

#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Tests
add_executable(testVectorLabel Tests/testVectorLabel.cpp)

