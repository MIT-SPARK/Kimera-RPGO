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
# Overall include directories
include_directories(
  include
)

# Maximum Clique Solver
add_library(max_clique_finder
    src/max_clique_finder/findClique.cpp
    src/max_clique_finder/findCliqueHeu.cpp 
    src/max_clique_finder/utils.cpp 
    src/max_clique_finder/graphIO.cpp
)

# Graph utils library
add_library(graph_utils
        src/graph_utils/graph_utils_functions.cpp
)
target_link_libraries(graph_utils
        gtsam
        max_clique_finder
)

add_library(RobustPGO
        src/RobustPGO/robust_pgo.cpp
)
target_link_libraries(RobustPGO
        graph_utils
)

###########################################################################
# Tests
add_subdirectory(tests)

###########################################################################
# Install
# install(TARGETS graph_utils DESTINATION lib)
# file(GLOB headers_graph_utils "include/graph_utils/*.h")
# install(FILES ${headers_graph_utils} DESTINATION include/graph_utils/)

# install(TARGETS max_clique_finder DESTINATION lib)
# file(GLOB headers_max_clique_finder "include/max_clique_finder/*.h")
# install(FILES ${headers_max_clique_finder} DESTINATION include/max_clique_finder/)

install(TARGETS RobustPGO DESTINATION lib)
file(GLOB headers_robust_PGO "include/RobustPGO/*.h")
install(FILES ${headers_robust_PGO} DESTINATION include/RobustPGO/)
