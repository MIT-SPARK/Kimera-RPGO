include_directories("${CMAKE_CURRENT_BINARY_DIR}")
gtsamAddTestsGlob(kimera_rpgoTests "test*.cpp" "" KimeraRPGO)