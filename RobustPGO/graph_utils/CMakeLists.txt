# Install headers
file(GLOB headers_graph_utils "*.h")
install(FILES ${headers_graph_utils} DESTINATION include/RobustPGO/graph_utils)

# Build tests
if (UNIT_TESTS)
	add_subdirectory(tests)
endif()