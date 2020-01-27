# Add source code for KimeraRPGO
target_sources(KimeraRPGO
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/findClique.h"
		"${CMAKE_CURRENT_LIST_DIR}/findClique.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/findCliqueHeu.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/graphIO.h"
		"${CMAKE_CURRENT_LIST_DIR}/graphIO.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/utils.cpp"
)
target_include_directories(KimeraRPGO PRIVATE ${CMAKE_CURRENT_LIST_DIR})