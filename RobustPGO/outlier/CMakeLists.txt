# Add source code for RobustPGO
target_sources(RobustPGO
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/OutlierRemoval.h"
		"${CMAKE_CURRENT_LIST_DIR}/pcm.h"
)
target_include_directories(RobustPGO PRIVATE ${CMAKE_CURRENT_LIST_DIR})
