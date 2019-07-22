# Add source code for RobustPGO
target_sources(RobustPGO
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/findClique.h"
		"${CMAKE_CURRENT_LIST_DIR}/findClique.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/findCliqueHeu.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/graphIO.h"
		"${CMAKE_CURRENT_LIST_DIR}/graphIO.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils.cpp"
)
target_include_directories(RobustPGO PRIVATE ${CMAKE_CURRENT_LIST_DIR})