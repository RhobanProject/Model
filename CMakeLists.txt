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

#Sources files
set(SOURCES_FILES
    CartWalk/CartWalk.cpp  
    CartWalk/linear_algebra.cpp  
    CartWalk/math_basics.cpp  
    CartWalk/math_log.cpp  
    CartWalk/SigmabanLeg.cpp
    CartWalk/CartWalkProxy.cpp  
    CartWalk/CartWalkGradient.cpp  
    LinearRegression/SimpleLinearRegression.cpp
)

#Tests
add_executable(testVectorLabel Tests/testVectorLabel.cpp ${SOURCES_FILES})
add_executable(testCartWalkProxy Tests/testCartWalkProxy.cpp ${SOURCES_FILES})
add_executable(testPlot Tests/testPlot.cpp ${SOURCES_FILES})
add_executable(testSimpleLinearRegression Tests/testSimpleLinearRegression.cpp ${SOURCES_FILES})
add_executable(testCartWalkGradient Tests/testCartWalkGradient.cpp ${SOURCES_FILES})

