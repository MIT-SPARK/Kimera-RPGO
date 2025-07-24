find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(
  _kimera_rpgo_bindings bindings/kimera_rpgo_bindings.cpp
)
target_link_libraries(_kimera_rpgo_bindings PRIVATE kimera_rpgo)
target_include_directories(
  _kimera_rpgo_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings
                              ${CMAKE_CURRENT_BINARY_DIR}/bindings
)

if(DEFINED SKBUILD AND "${SKBUILD}")
  install(TARGETS _kimera_rpgo_bindings DESTINATION ${PROJECT_NAME})
endif()
