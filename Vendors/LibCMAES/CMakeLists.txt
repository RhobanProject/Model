cmake_minimum_required(VERSION 2.8)
project(LibCMAES)

#Include sources directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#Include Eigen (linear algebra template library)
set(EIGEN_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../Eigen" CACHE STRING "Eigen lib directory")
include_directories(${EIGEN_DIRECTORY})

#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Sources files
set(SOURCES_FILES
    src/acovarianceupdate.cc
    src/cmasolutions.cc
    src/covarianceupdate.cc
    src/ipopcmastrategy.cc
    src/vdcmaupdate.cc
    src/bipopcmastrategy.cc
    src/cmastopcriteria.cc
    src/errstats.cc
    src/pwq_bound_strategy.cc
    src/cmaparameters.cc
    src/cmastrategy.cc
    src/esostrategy.cc
    #src/surrogatestrategy.cc
)

#Special flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

#Build Code as static library
add_library(cmaes STATIC ${SOURCES_FILES})

