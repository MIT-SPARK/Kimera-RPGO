add_library(rpgo_max_clique STATIC
    findClique.cpp
    findCliqueHeu.cpp
    graphIO.cpp
    utils.cpp
)
target_include_directories(rpgo_max_clique PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(rpgo_max_clique PRIVATE gtsam)
set_property(TARGET rpgo_max_clique PROPERTY POSITION_INDEPENDENT_CODE 1)
