target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/solver_config.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/solver.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/least_squares_solver.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gradient_solver.cpp
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/gnc_solver.cpp
)
