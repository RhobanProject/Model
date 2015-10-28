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
option(BUILD_LEPH_CODE_RHIO "Build Leph Code with RhIO" OFF)

#Includes directories lists
set(LEPHCODE_INCLUDE_DIRECTORIES "")

#Include sources directory
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR})

#Include Eigen (linear algebra template library)
set(EIGEN_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/Eigen" CACHE STRING "Eigen lib directory")
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${EIGEN_DIRECTORY})

#Include RBDL (dynamic and geometric robot library)
set(RBDL_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/RBDL" CACHE STRING "RBDL lib directory")
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${RBDL_DIRECTORY}/addons)
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${RBDL_DIRECTORY}/include)
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_BINARY_DIR}/Vendors/RBDL/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/Vendors/RBDL)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/Vendors/RBDL/urdfreader)

#Include LibCMAES (Black box non linear stochastic optimization)
set(LIBCMAES_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/LibCMAES" CACHE STRING "LibCMAES lib directory")
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${LIBCMAES_DIRECTORY})
link_directories(${CMAKE_CURRENT_BINARY_DIR}/Vendors/LibCMAES)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp" )

#Include LWPR (Fast non parametric regression)
set(LWPR_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/LWPR" CACHE STRING "LWPR lib directory")
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${LWPR_DIRECTORY}/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/Vendors/LWPR)

#Include LibGP (Gaussian Process regression C++ library)
set(LIBGP_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/LibGP" CACHE STRING "LibGP lib directory")
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${LIBGP_DIRECTORY}/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/Vendors/LibGP)

#Include SFML (graphic library)
set(SFML_DIRECTORY "SFML" CACHE STRING "SFML lib directory")

#Include ZeroMQ (network library)
set(ZMQ_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/ZeroMQ" CACHE STRING "ZMQ lib directory")

#Include RhIO (Rhoban Input Output Library) if needed
set(RHIO_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vendors/RhIO" CACHE STRING "RhIO lib directory")
if (BUILD_LEPH_CODE_RHIO)
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${RHIO_DIRECTORY}/include)
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${RHIO_DIRECTORY}/contrib)
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${RHIO_DIRECTORY}/Common)
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${RHIO_DIRECTORY}/Server)
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${RHIO_DIRECTORY}/Client)
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${ZMQ_DIRECTORY}/include)
    link_directories(${RHIO_DIRECTORY}/build)
    link_directories(${ZMQ_DIRECTORY}/lib)
endif (BUILD_LEPH_CODE_RHIO)

#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#Enable Warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#Sources files
set(SOURCES_FILES
    CartWalk/CartWalk.cpp  
    CartWalk/linear_algebra.cpp  
    CartWalk/math_basics.cpp  
    CartWalk/math_log.cpp  
    CartWalk/SigmabanLeg.cpp
    CartWalk/CartWalkProxy.cpp  
    CartWalk/CartWalkGradient.cpp  
    Concepts/HumanoidModelConcept.cpp
    Concepts/HumanoidSensorsModelConcept.cpp
    Concepts/FootStepDifferentiatorConcept.cpp
    Concepts/FootStepIntegratorConcept.cpp
    Concepts/FallDetectorConcept.cpp
    LinearRegression/SimpleLinearRegression.cpp
    LinearRegression/MedianLinearRegression.cpp
    Gradient/FiniteDifferenceGradient.cpp
    Utils/GeometricMedian.cpp
    Utils/Combination.cpp
    Utils/CircularBuffer.cpp
    Utils/Chrono.cpp
    Utils/Scheduling.cpp
    Utils/Differentiation.cpp
    Utils/NewtonBinomial.cpp
    Utils/LWPRUtils.cpp
    Utils/RandomWalk.cpp
    Ncurses/InterfaceCLI.cpp
    Model/Model.cpp
    Model/HumanoidModel.cpp
    Model/HumanoidFloatingModel.cpp
    Model/HumanoidFixedModel.cpp
    Model/HumanoidFixedPressureModel.cpp
    Model/InverseKinematics.cpp
    Model/RBDLRootUpdate.cpp
    Model/RBDLClosedLoop.cpp
    Model/NullSpace.cpp
    StaticWalk/StaticWalk.cpp
    Spline/Polynom.cpp
    Spline/Spline.cpp
    Spline/SmoothSpline.cpp
    Spline/LinearSpline.cpp
    Spline/CubicSpline.cpp
    Spline/FittedSpline.cpp
    Spline/PolyFit.cpp
    LegIK/LegIK.cpp
    IKWalk/IKWalk.cpp
    Utils/IterativeLearningControl.cpp
    TimeSeries/SeriesUtils.cpp
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
    testCubicSpline
    testSmoothSpline
    testScheduling
    testFittedSpline
    testDifferentiation
    testGaussianProcess
    testMetaSeries
    testTimeSeries
    testMetaParameter
    testConcept
    testRegressionLWPR
    testLWPR
    testLWPRAdvanced
    testIterativeLearningControl
    testRandomWalk
)

#Applications main files
set(APPS_FILES
    appSplineGenerationCircle
    appTestLearningFromLogs
    appExperimentOdometry
)

#LephCode linked libraries
set(CODE_LIBRARIES
    ncurses
    rbdl
    rbdl_urdfreader
    cmaes
    lwpr
    gp
)

#Build LibCMAES vendors
add_subdirectory(${LIBCMAES_DIRECTORY})

#Build LWPR vendors
add_subdirectory(${LWPR_DIRECTORY})

#Build LibGP vendors
add_subdirectory(${LIBGP_DIRECTORY})

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
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${SFML_DIRECTORY}/include)
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
        testIKWalk
        testZeroMomentPoint
        testHumanoidPressureModel
    )
    set(APPS_FILES ${APPS_FILES}
        appViewerModelSplines
        appViewerModelLogsOld
        appViewerModelLogs
        appViewerModelZMP
        appStaticTrajectoryOptimizationTorques
        appTestLegTorques
    )
    set(CODE_LIBRARIES ${CODE_LIBRARIES} 
        sfml-graphics
        sfml-window
        sfml-system
        GL
        GLU
    )
endif (BUILD_LEPH_CODE_VIEWER)

#Build App RhIO Viewer 
if (BUILD_LEPH_CODE_RHIO)
    set(APPS_FILES ${APPS_FILES}
        appRhIOViewerModel
    )
    set(CODE_LIBRARIES ${CODE_LIBRARIES} 
        RhIOClient
        zmq
    )
endif (BUILD_LEPH_CODE_RHIO)

#Include need directories
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} CACHE STRING "LephCode include directories")
include_directories(${LEPHCODE_INCLUDE_DIRECTORIES})

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

