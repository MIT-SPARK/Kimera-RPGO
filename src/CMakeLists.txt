target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/rpgo.cpp
)

add_subdirectory(utils)
add_subdirectory(solver)
add_subdirectory(pcm)
