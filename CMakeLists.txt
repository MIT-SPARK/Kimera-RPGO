cmake_minimum_required (VERSION 3.5)
project(kimera_rpgo VERSION 1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

IF(APPLE)
    # Fix linking on 10.14+. See https://stackoverflow.com/questions/54068035
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

find_package(GTSAM REQUIRED)

###########################################################################
# Compile
add_library(KimeraRPGO SHARED
  src/GenericSolver.cpp
  src/RobustSolver.cpp
  src/Logger.cpp
  include/KimeraRPGO/SolverParams.h )
include_directories(include)
# Add source code for max clique finder
add_subdirectory(include/KimeraRPGO/max_clique_finder)
# Add source for outlier rejection stuff
add_subdirectory(include/KimeraRPGO/outlier)
# Add source for utils
add_subdirectory(include/KimeraRPGO/utils)
add_subdirectory(src/utils)

target_include_directories(KimeraRPGO PUBLIC
  ${GTSAM_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  # INSTALL
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(KimeraRPGO
  PUBLIC
    gtsam
  PRIVATE
    rpgo_max_clique
)

target_compile_options(KimeraRPGO
  PRIVATE -Wall -pipe
)

###########################################################################
# Define executables
add_executable(RpgoReadG2o examples/RpgoReadG2o.cpp)
target_link_libraries(RpgoReadG2o KimeraRPGO)
add_executable(RpgoReadG2oIncremental examples/RpgoReadG2oIncremental.cpp)
target_link_libraries(RpgoReadG2oIncremental KimeraRPGO)
add_executable(GenerateTrajectories examples/GenerateTrajectories.cpp)
target_link_libraries(GenerateTrajectories gtsam)

###########################################################################
# Tests
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/UnitTesting.cmake)
add_subdirectory(tests)
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
configure_file(tests/test_config.h.in tests/test_config.h)

###########################################################################
# Installation
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/KimeraRPGO)

install(TARGETS KimeraRPGO
  EXPORT kimera-rpgo-export
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(EXPORT kimera-rpgo-export
  FILE
    KimeraRPGOTargets.cmake
  DESTINATION
    ${INSTALL_CONFIGDIR}
)
# Install header files
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/KimeraRPGOConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
# Create KimeraRPGOConfig.cmake with extra info from KimeraRPGOConfig.cmake.in
# This file is necessary to find_package the library KimeraRPGO.
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/KimeraRPGOConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/KimeraRPGOConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR})
## Install the config and configversion
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/KimeraRPGOConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/KimeraRPGOConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

################################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "CMAKE_CXX_COMPILER_ID type                : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build flags                                               ")
if(NOT MSVC AND NOT XCODE_VERSION)
    message(STATUS "  Build type                              : ${CMAKE_BUILD_TYPE}")
    if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release" AND NOT ${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo")
      message(WARNING "  Build type is not in RELEASE, this will slow down the code.")
    endif()
    message(STATUS "  C compilation flags (Release)           : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
    message(STATUS "  C++ compilation flags (Release)         : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif()
