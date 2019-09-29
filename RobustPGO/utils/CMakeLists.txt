# Add source code for RobustPGO
target_sources(RobustPGO
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/geometry_utils.h"
		"${CMAKE_CURRENT_LIST_DIR}/graph_utils.h"
		"${CMAKE_CURRENT_LIST_DIR}/graph_utils.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/type_utils.h"
)
target_include_directories(RobustPGO PRIVATE ${CMAKE_CURRENT_LIST_DIR})
