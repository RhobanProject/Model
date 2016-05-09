# Created by Manuel Blum on 2011-05-25.
# Copyright 2013 University of Freiburg.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(libgp CXX C)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCLSQUARE -fPIC")

OPTION(BUILD_LIBGP_TESTS "Build tests" OFF)
OPTION(BUILD_LIBGP_EXAMPLES "Build examples" OFF)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

# ----- Add include directories -----
INCLUDE_DIRECTORIES(${EIGEN_DIRECTORY})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ----- Add sources -----
INCLUDE("Sources.cmake")

# ----- Add library target -----
ADD_LIBRARY(gp ${LIBGP_SRC} ${LIBGP_INTERFACES})

# ----- Add example targets -----
IF(BUILD_LIBGP_EXAMPLES)
  ADD_SUBDIRECTORY(examples)
ENDIF()

# ----- Testing -----
IF(BUILD_LIBGP_TESTS)
  ADD_SUBDIRECTORY(tests)
ENDIF()

