# Install headers
file(GLOB headers_graph_utils "*.h")
install(FILES ${headers_graph_utils} DESTINATION include/RobustPGO/graph_utils)

# Build tests
add_subdirectory(tests)