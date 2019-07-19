# Add source code for RobustPGO
target_sources(RobustPGO
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/pcm.h"
		"${CMAKE_CURRENT_LIST_DIR}/pcm_distance.h"
)
target_include_directories(RobustPGO PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Build tests
if (UNIT_TESTS)
	add_subdirectory(tests)
endif()