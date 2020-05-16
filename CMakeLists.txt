cmake_minimum_required (VERSION 3.5)
project(kimera_rpgo VERSION 1.0 LANGUAGES CXX)

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
add_library(kimera_rpgo SHARED
  include/kimera_rpgo/logger.h
  src/GenericSolver.cpp
  src/RobustSolver.cpp
  include/kimera_rpgo/SolverParams.h )
include_directories(include)
# Add source code for max clique finder
add_subdirectory(include/kimera_rpgo/max_clique_finder)
# Add source for outlier rejection stuff
add_subdirectory(include/kimera_rpgo/outlier)
# Add source for utils
add_subdirectory(include/kimera_rpgo/utils)
add_subdirectory(src/utils)

target_include_directories(kimera_rpgo PUBLIC
  ${GTSAM_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  # INSTALL
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(kimera_rpgo
  PUBLIC
    Boost::boost
    gtsam
    gtsam_unstable
)

target_compile_options(kimera_rpgo
  PRIVATE -Wall -pipe
  PRIVATE -march=native
)

###########################################################################
# Define executables
add_executable(RpgoReadG2o examples/RpgoReadG2o.cpp)
target_link_libraries(RpgoReadG2o kimera_rpgo)
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
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create kimera_rpgoConfig.cmake with extra info from kimera_rpgoConfig.cmake.in
# This file is necessary to find_package the library kimera_rpgo.
set(INSTALL_CONFIGDIR lib/cmake/kimera_rpgo)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/kimera_rpgoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR})

include(GNUInstallDirs)
install(TARGETS kimera_rpgo
  EXPORT kimera-rpgo-export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include # We need this right?
  RUNTIME DESTINATION bin
)
install(EXPORT kimera-rpgo-export
  FILE
    kimera_rpgoTargets.cmake
  DESTINATION
    ${INSTALL_CONFIGDIR}
)
# Install header files
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

## Install the config and configversion
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

export(TARGETS kimera_rpgo FILE kimera_rpgoTargets.cmake)

################################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "CMAKE_CXX_COMPILER_ID type                : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION                : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build flags                                               ")
if(NOT MSVC AND NOT XCODE_VERSION)
    message(STATUS "  Build type                              : ${CMAKE_BUILD_TYPE}")
    if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
      message(WARNING "  Build type is not in RELEASE, this will slow down the code.")
    endif()
    message(STATUS "  C compilation flags (Release)           : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
    message(STATUS "  C++ compilation flags (Release)         : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif()