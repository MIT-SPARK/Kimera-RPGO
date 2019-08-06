cmake_minimum_required (VERSION 3.5)
project(RobustPGO VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14) # TODO: remove this later and use target_compile_options
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
add_library(RobustPGO STATIC
  RobustPGO/logger.h
  RobustPGO/GenericSolver.cpp
  RobustPGO/RobustSolver.cpp
  RobustPGO/SolverParams.h )

# Add source code for max clique finder
include(RobustPGO/max_clique_finder/CMakeLists.txt)
# Add source for outlier rejection stuff
include(RobustPGO/outlier/CMakeLists.txt)
# Add source for utils
include(RobustPGO/utils/CMakeLists.txt)

target_link_libraries(RobustPGO
  PUBLIC
    Boost::boost
    gtsam
    gtsam_unstable
)

target_include_directories(RobustPGO
  PUBLIC
    ${GTSAM_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

###########################################################################
# Define executables
add_executable(RpgoReadG2o examples/RpgoReadG2o.cpp)
target_link_libraries(RpgoReadG2o RobustPGO)

###########################################################################
# Tests
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/UnitTesting.cmake)
add_subdirectory(tests)
add_subdirectory(RobustPGO/outlier/tests)
add_subdirectory(RobustPGO/utils/tests)
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
configure_file(tests/test_config.h.in tests/test_config.h)

###########################################################################
# Installation
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/RobustPGO)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/RobustPGOConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create RobustPGOConfig.cmake with extra info from RobustPGOConfig.cmake.in
# This file is necessary to find_package the library RobustPGO.
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/RobustPGOConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/RobustPGOConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

export(TARGETS RobustPGO FILE RobustPGOTargets.cmake)
export(PACKAGE RobustPGO)
