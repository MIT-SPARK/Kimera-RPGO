include_directories("${CMAKE_CURRENT_BINARY_DIR}")
gtsamAddTestsGlob(RobustPGOTests "test*.cpp" "" RobustPGO)