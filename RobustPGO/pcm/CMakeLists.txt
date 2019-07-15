# Install headers
file(GLOB headers_pcm "*.h")
install(FILES ${headers_pcm} DESTINATION include/RobustPGO/pcm)

# Build tests
if (UNIT_TESTS)
	add_subdirectory(tests)
endif()