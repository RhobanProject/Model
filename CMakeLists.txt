###
### Quentin Rouxel
### Thesis (2014-201?)
###

cmake_minimum_required(VERSION 3.0)
project(Thesis)

#Option for compiling the tests and apps
option(BUILD_LEPH_CODE_TESTS "Build Leph Code Tests" OFF)
option(BUILD_LEPH_CODE_APPS "Build Leph Code Apps" OFF)
option(BUILD_LEPH_CODE_VIEWER "Build Leph Code Viewer (SFML)" OFF)

#Include sources directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Include Eigen (linear algebra template library)
set(EIGEN_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/Eigen" CACHE STRING "Eigen lib directory")
include_directories(${EIGEN_DIRECTORY})

#Include RBDL (dynamic and geometric robot library)
set(RBDL_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/RBDL" CACHE STRING "RBDL lib directory")
include_directories(${RBDL_DIRECTORY}/addons)
include_directories(${RBDL_DIRECTORY}/include)
include_directories(${CMAKE_BINARY_DIR}/Vendors/RBDL/include)
link_directories(${CMAKE_BINARY_DIR}/Vendors/RBDL)
link_directories(${CMAKE_BINARY_DIR}/Vendors/RBDL/urdfreader)

#Include LibCMAES (Black box non linear stochastic optimization)
set(LIBCMAES_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/LibCMAES" CACHE STRING "LibCMAES lib directory")
include_directories(${LIBCMAES_DIRECTORY})
link_directories(${CMAKE_BINARY_DIR}/Vendors/LibCMAES)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp" )

#Include SFML (graphic library)
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
    Utils/Chrono.cpp
    Utils/Scheduling.cpp
    Ncurses/InterfaceCLI.cpp
    Model/Model.cpp
    Model/HumanoidModel.cpp
    Model/HumanoidFloatingModel.cpp
    Model/HumanoidFixedModel.cpp
    Model/InverseKinematics.cpp
    Model/RBDLRootUpdate.cpp
    Model/RBDLClosedLoop.cpp
    Model/NullSpace.cpp
    StaticWalk/StaticWalk.cpp
    Spline/Polynom.cpp
    Spline/Spline.cpp
    Spline/SmoothSpline.cpp
    Spline/FittedSpline.cpp
    LegIK/LegIK.cpp
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
    testModel
    testLevenbergMarquardt
    testChrono
    testSmoothSpline
    testScheduling
    testFittedSpline
)

#Applications main files
set(APPS_FILES
    appSplineGenerationCircle
)

#LephCode linked libraries
set(CODE_LIBRARIES
    ncurses
    rbdl
    rbdl_urdfreader
    cmaes
)

#Build LibCMAES vendors
add_subdirectory(${LIBCMAES_DIRECTORY})

#Build RBDL vendors
set(RBDL_BUILD_ADDON_URDFREADER ON CACHE BOOL "Build RBDL URDF Addon")
add_subdirectory(${RBDL_DIRECTORY})

#Add URDF model symbolic link
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/sigmaban.urdf sigmaban.urdf")
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/grosban.urdf grosban.urdf")

#Viewer (SFML) build option and files
if (BUILD_LEPH_CODE_VIEWER)
    include_directories(${SFML_DIRECTORY}/include)
    link_directories(${SFML_DIRECTORY}/lib)
    set(SOURCES_FILES ${SOURCES_FILES} 
        Viewer/ModelViewer.cpp
        Viewer/ModelDraw.cpp
    )
    set(TESTS_FILES ${TESTS_FILES} 
        testModelViewer
        testInverseKinematics
        testStaticWalk
        testRBDLRootUpdate
        testSigmabanFixedModel
        testNullSpace
        testLegIK
        testModelTorques
    )
    set(APPS_FILES ${APPS_FILES}
        appViewerModelSplines
        appViewerModelLogs
        appStaticTrajectoryOptimizationTorques
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

#Build Apps
if (BUILD_LEPH_CODE_APPS)
    foreach (APP ${APPS_FILES})
        add_executable(${APP} Apps/${APP}.cpp)
        target_link_libraries(${APP} LephCode)
    endforeach ()
endif (BUILD_LEPH_CODE_APPS)

