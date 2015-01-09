cmake_minimum_required(VERSION 2.8)
project(ExperimentCartWalkGradientOptim)

#Rhoban dependencies repositories paths for headers inclusion
set(RHOBAN_MATH_DIRECTORY
    "/home/rhoban/Math/" CACHE STRING "RhobanMath directory")
set(RHOBAN_COMMON_DIRECTORY
    "/home/rhoban/Common/" CACHE STRING "RhobanCommon directory")
set(RHOBAN_UTILS_DIRECTORY
    "/home/rhoban/Utils/" CACHE STRING "RhobanUtils directory")

#Rhoban C++ SDK directory path
set(RHOBAN_SDK_DIRECTORY "/home/rhoban/Edu/sdk/cpp" CACHE STRING "RhobanSdk cpp directory")

#Path to a SDK build directory (with SDK, Utils, Common and library binaries)
set(SDK_BUILD_DIRECTORY "/home/rhoban/Edu/build" CACHE STRING "RhobanSdkBuild directory")

#Experiment source files
set(SOURCES
    main.cpp
)

#Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   
# Header includes
include_directories(
    ${RHOBAN_MATH_DIRECTORY}
    ${RHOBAN_COMMON_DIRECTORY}
    ${RHOBAN_UTILS_DIRECTORY}
    ${RHOBAN_SDK_DIRECTORY}
    ${RHOBAN_SDK_DIRECTORY}/rhoban
    ${SDK_BUILD_DIRECTORY}/common/
)

link_directories(
    ${SDK_BUILD_DIRECTORY}
    ${SDK_BUILD_DIRECTORY}/common
    ${SDK_BUILD_DIRECTORY}/utils
    ${SDK_BUILD_DIRECTORY}/utils/yaml-cpp
    ${SDK_BUILD_DIRECTORY}/utils/xml/TinyXml
)

add_executable(ExperimentCartWalkGradientOptim ${SOURCES})
target_link_libraries(ExperimentCartWalkGradientOptim 
    RhobanSDKMain 
    RhobanCommon 
    RhobanUtils
    pthread
    yaml-cpp
    TinyXml
)

