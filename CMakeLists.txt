cmake_minimum_required (VERSION 3.5)
project(KimeraRPGO VERSION 1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

IF(APPLE)
    # Fix linking on 10.14+. See https://stackoverflow.com/questions/54068035
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

###########################################################################
# Find Boost
find_package(Boost REQUIRED COMPONENTS filesystem regex timer date_time program_options)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

###########################################################################
# Compile
add_library(KimeraRPGO SHARED
  KimeraRPGO/logger.h
  KimeraRPGO/GenericSolver.cpp
  KimeraRPGO/RobustSolver.cpp
  KimeraRPGO/SolverParams.h )

# Add source code for max clique finder
include(KimeraRPGO/max_clique_finder/CMakeLists.txt)
# Add source for outlier rejection stuff
include(KimeraRPGO/outlier/CMakeLists.txt)
# Add source for utils
include(KimeraRPGO/utils/CMakeLists.txt)

target_link_libraries(KimeraRPGO
  PUBLIC
    Boost::boost
    gtsam
    gtsam_unstable
)

target_include_directories(KimeraRPGO
  PUBLIC
    ${GTSAM_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

###########################################################################
# Define executables
add_executable(RpgoReadG2o examples/RpgoReadG2o.cpp)
target_link_libraries(RpgoReadG2o KimeraRPGO)
add_executable(GenerateTrajectories examples/GenerateTrajectories.cpp)
target_link_libraries(GenerateTrajectories gtsam)

###########################################################################
# Tests
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/UnitTesting.cmake)
add_subdirectory(tests)
add_subdirectory(KimeraRPGO/outlier/tests)
add_subdirectory(KimeraRPGO/utils/tests)
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
configure_file(tests/test_config.h.in tests/test_config.h)

###########################################################################
# Installation
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/KimeraRPGO)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/KimeraRPGOConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create KimeraRPGOConfig.cmake with extra info from KimeraRPGOConfig.cmake.in
# This file is necessary to find_package the library KimeraRPGO.
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/KimeraRPGOConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/KimeraRPGOConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

export(TARGETS KimeraRPGO FILE KimeraRPGOTargets.cmake)
export(PACKAGE KimeraRPGO)

include(GNUInstallDirs)
install(TARGETS KimeraRPGO
    EXPORT kimera-rpgo-export
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(EXPORT kimera-rpgo-export
  FILE
    KimeraRPGOTargets.cmake
    #NAMESPACE
    #NAKimeraRPGO::
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/KimeraRPGO
)
