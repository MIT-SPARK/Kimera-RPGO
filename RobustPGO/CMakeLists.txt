add_library(RobustPGO
    max_clique_finder/findClique.cpp
    max_clique_finder/findCliqueHeu.cpp 
    max_clique_finder/utils.cpp 
    max_clique_finder/graphIO.cpp
    graph_utils/graph_utils.h
    pcm/pcm.h
    GenericSolver.cpp
    RobustPGO.cpp
)

target_link_libraries(RobustPGO
    gtsam
)

install(TARGETS RobustPGO EXPORT RPGO-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
file(GLOB headers_robust_PGO "*.h")
install(FILES ${headers_robust_PGO} DESTINATION include/RobustPGO)
add_subdirectory(max_clique_finder)
add_subdirectory(graph_utils)
add_subdirectory(pcm)