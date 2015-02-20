###
### Quentin Rouxel
### Thesis (2014-201?)
###

cmake_minimum_required(VERSION 2.8)
project(Thesis)

#Option for compiling the tests and apps
option(BUILD_LEPH_CODE_TESTS "Build Leph Code Tests" OFF)
option(BUILD_LEPH_CODE_APPS "Build Leph Code Apps" OFF)
option(BUILD_LEPH_CODE_VIEWER "Build Leph Code Viewer (SFML)" OFF)

#Include sources directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Include Eigen (linear algebra template library)
set(EIGEN_DIRECTORY "Eigen" CACHE STRING "Eigen lib directory")
include_directories(${EIGEN_DIRECTORY})

#Include SFML
set(SFML_DIRECTORY "SFML" CACHE STRING "SFML lib directory")

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
    LinearRegression/MedianLinearRegression.cpp
    Gradient/FiniteDifferenceGradient.cpp
    Utils/GeometricMedian.cpp
    Utils/Combination.cpp
    Utils/CircularBuffer.cpp
    Ncurses/InterfaceCLI.cpp
)

#Tests main files
set(TESTS_FILES
    testVectorLabel
    testCartWalkProxy
    testPlot
    testSimpleLinearRegression
    testCartWalkGradient
    testFiniteDifferenceGradient
    testReweightedLinearRegression
    testGeometricMedian
    testMedianLinearRegression
    testCombination
    testInterfaceCLI
    testCircularBuffer
    benchVectorLabel
    testMatrixLabel
)

#Applications main files
set(APPS_FILES
    appWalkExperimentLogsProcessing
    appPlotter
)

#LephCode linked libraries
set(CODE_LIBRARIES
    ncurses
)

#Viewer (SFML) build option and files
if (BUILD_LEPH_CODE_VIEWER)
    include_directories(${SFML_DIRECTORY}/include)
    link_directories(${SFML_DIRECTORY}/lib)
    set(SOURCES_FILES ${SOURCES_FILES} 
        Viewer/ModelViewer.cpp
    )
    set(TESTS_FILES ${TESTS_FILES} 
        testModelViewer
    )
    set(CODE_LIBRARIES ${CODE_LIBRARIES} 
        sfml-graphics
        sfml-window
        sfml-system
        GL
        GLU
    )
endif (BUILD_LEPH_CODE_VIEWER)

#Build Code as static library
add_library(LephCode STATIC ${SOURCES_FILES})
target_link_libraries(LephCode ${CODE_LIBRARIES})

#Build Tests
if (BUILD_LEPH_CODE_TESTS)
    foreach (TEST ${TESTS_FILES})
        add_executable(${TEST} Tests/${TEST}.cpp)
        target_link_libraries(${TEST} LephCode)
    endforeach ()
endif (BUILD_LEPH_CODE_TESTS)

