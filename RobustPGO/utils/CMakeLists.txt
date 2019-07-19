# Install headers
file(GLOB headers_utils "*.h")
install(FILES ${headers_utils} DESTINATION include/RobustPGO/utils)

# Build tests
if (UNIT_TESTS)
	add_subdirectory(tests)
endif()