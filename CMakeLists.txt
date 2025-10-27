cmake_minimum_required (VERSION 3.16)
project(kimera_rpgo VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # from: https://www.kitware.com//cmake-and-the-default-build-type/
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

if(NOT CMAKE_INTERPROCEDURAL_OPTIMIZATION)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON CACHE STRING "Turn on -flto" FORCE)
endif()

# Options
option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(KIMERA_RPGO_BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

find_package(GTSAM REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC nlohmann_json::nlohmann_json gtsam)
if(NOT BUILD_SHARED_LIBS)
  set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
endif()

add_subdirectory(src)

include(CTest)
if (BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

if (KIMERA_RPGO_BUILD_PYTHON_BINDINGS)
  message(STATUS "RPGO-tools Python binding will be built.")
  add_subdirectory(python)
endif()

###########################################################################
# Installation
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(kimera_rpgo::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_rpgoConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
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
