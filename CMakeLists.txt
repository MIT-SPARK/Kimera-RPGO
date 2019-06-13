cmake_minimum_required (VERSION 3.5)
project(RobustPGO)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fPIC")
add_definitions(-DLOG_DIR="${PROJECT_SOURCE_DIR}/..")

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

# for unittest examples
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
include(GtsamTesting)

###########################################################################
# Find Boost
find_package(Boost COMPONENTS serialization filesystem system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
message("Boost Libraries: ${Boost_LIBRARIES}")

###########################################################################
include_directories(BEFORE
		${PROJECT_SOURCE_DIR} 
		${PROJECT_BINARY_DIR}
)# So we can include generated config header files

add_subdirectory(RobustPGO)

###########################################################################
# Tests
add_subdirectory(tests)
