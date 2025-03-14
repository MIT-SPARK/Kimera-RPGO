cmake_minimum_required(VERSION 3.14)
project(kimera_rpgo_python)

include(FetchContent)
FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11.git GIT_TAG v2.11.1)
FetchContent_Populate(pybind11)
add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR} EXCLUDE_FROM_ALL)

pybind11_add_module(
  _kimera_rpgo_bindings bindings/kimera_rpgo_bindings.cpp
)
target_link_libraries(_kimera_rpgo_bindings PRIVATE kimera_rpgo)
target_include_directories(
  _kimera_rpgo_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings
                              ${CMAKE_CURRENT_BINARY_DIR}/bindings
)
