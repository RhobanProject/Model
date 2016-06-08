###
### Quentin Rouxel
### Thesis (2014-201?)
###

cmake_minimum_required(VERSION 2.8)
project(Thesis)

#Deps dependencies
find_package(Deps)
deps_add_libraries()

#Option for compiling the tests and apps
option(BUILD_LEPH_CODE_TESTS "Build Leph Code Tests" OFF)
option(BUILD_LEPH_CODE_APPS "Build Leph Code Apps" OFF)
option(BUILD_LEPH_CODE_VIEWER "Build Leph Code Viewer (SFML)" OFF)
option(BUILD_LEPH_CODE_RHIO "Build Leph Code with RhIO" OFF)
option(BUILD_LEPH_CODE_RHAL "Build Leph Code with RhAL" OFF)

#Includes directories lists
set(LEPHCODE_INCLUDE_DIRECTORIES "")

#Include sources directory
set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${CMAKE_CURRENT_SOURCE_DIR})

#OpenMP flags for LibCMAES
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp" )

#Include SFML (graphic library)
set(SFML_DIRECTORY "SFML" CACHE STRING "SFML lib directory")

#Include RhAL (Rhoban Abstraction Layer Library) if needed
if (BUILD_LEPH_CODE_RHAL)
    add_definitions(-DBUILD_RHAL_RHIO_BINDING)
endif (BUILD_LEPH_CODE_RHAL)

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
    Utils/RandomVelocitySpline.cpp
    Utils/IterativeLearningControl.cpp
    Utils/LWPRInputsOptimization.cpp
    Utils/EncoderFilter.cpp
    Utils/ComputeModelData.cpp
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
    Model/ForwardSimulation.cpp
    Model/MotorModel.cpp
    StaticWalk/StaticWalk.cpp
    Spline/Polynom.cpp
    Spline/Spline.cpp
    Spline/SmoothSpline.cpp
    Spline/LinearSpline.cpp
    Spline/CubicSpline.cpp
    Spline/FittedSpline.cpp
    Spline/PolyFit.cpp
    TrajectoryGeneration/TrajectoryGeneration.cpp
    TrajectoryGeneration/TrajectoryUtils.cpp
    LegIK/LegIK.cpp
    IKWalk/IKWalk.cpp
    QuinticWalk/QuinticWalk.cpp
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
    testRandomVelocitySpline
    testLWPRInputsOptimization
    testEuler
    testAxisAngle
    testEncoderFilter
    testMapSeries
    testCameraModel
)

#Applications main files
set(APPS_FILES
    appSplineGenerationCircle
    appTestLearningFromLogs
    appExperimentOdometry
    appOdometryOffline
    appTrajectoriesFitting
    appStatsDynaban
)

#LephCode linked libraries
set(CODE_LIBRARIES
    ncurses
    console_bridge 
    urdfdom_world 
    urdfdom_model
    ${DEPS_LIBRARIES}
)

#Add URDF model symbolic link
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/sigmaban_metabot.urdf sigmaban.urdf")
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/grosban.urdf grosban.urdf")
#Add RhAL config symbolic link
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/django_rhal.json django_rhal.json")
execute_process(COMMAND "/bin/bash" "-c" 
    "cd ${CMAKE_BINARY_DIR}; ln -f -s ${CMAKE_CURRENT_SOURCE_DIR}/Data/chewbacca_rhal.json chewbacca_rhal.json")

#Viewer (SFML) build option and files
if (BUILD_LEPH_CODE_VIEWER)
    set(LEPHCODE_INCLUDE_DIRECTORIES ${LEPHCODE_INCLUDE_DIRECTORIES} ${SFML_DIRECTORY}/include)
    link_directories(${SFML_DIRECTORY}/lib)
    set(SOURCES_FILES ${SOURCES_FILES} 
        Viewer/ModelViewer.cpp
        Viewer/ModelDraw.cpp
        TrajectoryGeneration/TrajectoryDisplay.cpp
    )
    set(TESTS_FILES ${TESTS_FILES} 
        testModelViewer
        testInverseKinematics
        testStaticWalk
        testRBDLRootUpdate
        testHumanoidFixedModel
        testNullSpace
        testLegIK
        testModelTorques
        testIKWalk
        testZeroMomentPoint
        testHumanoidPressureModel
        testTrunkFootIK
        testForwardSimulation
        testQuinticWalk
    )
    set(APPS_FILES ${APPS_FILES}
        appViewerModelSplines
        appViewerModelLogsOld
        appViewerModelLogs
        appViewerModelZMP
        appDynamicTrajectoryOptimizationTorques
        appTrajectoriesViewer
        appTestLegTorques
        appExperimentKick
        appExperimentKick2
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

#Build Code as shared library
add_library(LephCode SHARED ${SOURCES_FILES})
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

#Build App RhAL Motion 
if (BUILD_LEPH_CODE_RHAL)
    set(APPS_RHAL_SOURCES_FILES
        RhAL/RhALUtils
    )
    set(APPS_RHAL_LIBRARIES
        RhAL 
        RhIO 
        serial 
        zmq
    )
    set(APPS_RHAL_FILES
        appRhALTrajectory
    )
    foreach (APP ${APPS_RHAL_FILES})
        add_executable(${APP} Apps/${APP}.cpp ${APPS_RHAL_SOURCES_FILES})
        target_link_libraries(${APP} LephCode ${APPS_RHAL_LIBRARIES})
    endforeach ()
endif (BUILD_LEPH_CODE_RHAL)

