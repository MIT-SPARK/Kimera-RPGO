# Add source code for KimeraRPGO
target_sources(KimeraRPGO
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/OutlierRemoval.h"
		"${CMAKE_CURRENT_LIST_DIR}/pcm.h"
)
target_include_directories(KimeraRPGO PRIVATE ${CMAKE_CURRENT_LIST_DIR})
