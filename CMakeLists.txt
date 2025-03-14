cmake_minimum_required (VERSION 3.14)
project(kimera_rpgo VERSION 2.0.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -fPIC)

# Options
option(KIMERA_RPGO_BUILD_TESTS "Build tests" ON)
option(KIMERA_RPGO_BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

# Cache Variables
if (NOT KIMERA_RPGO_PYTHON_VERSION)
    set(KIMERA_RPGO_PYTHON_VERSION "" CACHE STRING "Python version to use for kimera_rpgo bindings.")
endif ()

find_package(GTSAM REQUIRED)
find_package(nlohmann_json REQUIRED)

###########################################################################
# Compile
add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC ${GTSAM_INCLUDE_DIR}
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         # INSTALL
         $<INSTALL_INTERFACE:include>
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

target_link_libraries(
    ${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json gtsam
)

add_subdirectory(src)

###########################################################################
# Define executables

###########################################################################
# Tests
if (KIMERA_RPGO_BUILD_TESTS)
  add_subdirectory(tests)
  set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
  configure_file(tests/test_config.h.in tests/test_config.h)
endif()

##########################################################################
# Python Bindings
if (KIMERA_RPGO_BUILD_PYTHON_BINDINGS)
  message(STATUS "RPGO-tools Python binding will be built.")
  add_subdirectory(python)
endif()

###########################################################################
# Installation
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(
  kimera_rpgo::${PROJECT_NAME} ALIAS ${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfigVersion.cmake VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/kimera_rpgoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kimera_rpgo
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT kimera_rpgo-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT kimera_rpgo-targets
  FILE kimera_rpgoTargets.cmake
  NAMESPACE kimera_rpgo::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kimera_rpgo
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kimera_rpgo
)
