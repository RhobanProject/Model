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


set(HEADERS
	src/acovarianceupdate.h
	src/cmastrategy.h
	src/esostrategy.h
	src/pli.h
	src/bipopcmastrategy.h
	src/contour.h
	src/genopheno.h
	src/pwq_bound_strategy.h
	src/candidate.h
	src/covarianceupdate.h
	src/ipopcmastrategy.h
	src/scaling.h
	src/cmaes.h
	src/eigenmvn.h
	src/llogging.h
	src/surrcmaes.h
	src/cmaparameters.h
	src/eo_matrix.h
	src/noboundstrategy.h
	src/surrogatestrategy.h
	src/cmasolutions.h
	src/errstats.h
	src/opti_err.h
	src/vdcmaupdate.h
	src/cmastopcriteria.h
	src/esoptimizer.h
	src/parameters.h
	libcmaes_config.h
)

#Special flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

#Build Code as static library
add_library(cmaes STATIC ${SOURCES_FILES})

install(TARGETS cmaes DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/cmaes/)
