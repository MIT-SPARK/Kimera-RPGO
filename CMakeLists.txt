cmake_minimum_required (VERSION 3.5)
project(RobustPGO)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

option(UNIT_TESTS "Enable/Disable including unit tests." ON)

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest examples
if (UNIT_TESTS)
	# for unittest scripts
	set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
endif()

###########################################################################
# Find Boost
find_package(Boost 1.58 COMPONENTS filesystem program_options timer REQUIRED)

###########################################################################
include_directories(
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${GTSAM_LIBRARY_DIRS}
)

add_subdirectory(RobustPGO)

###########################################################################
# Define executables
add_executable(RpgoReadG2o examples/RpgoReadG2o.cpp)
target_link_libraries(RpgoReadG2o RobustPGO)

###########################################################################
# Tests
if (UNIT_TESTS)
	add_subdirectory(tests)
	set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
	configure_file(tests/test_config.h.in test_config.h)
endif()