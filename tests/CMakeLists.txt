find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
configure_file(test_config.h.in test_config.h)

add_executable(test_${PROJECT_NAME} test_gnc.cpp test_gradient_descent.cpp
                                    test_least_squares.cpp test_pcm.cpp)
target_include_directories(
  test_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                               ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_${PROJECT_NAME} PRIVATE ${PROJECT_NAME}
                                                   GTest::gtest_main)
gtest_add_tests(TARGET test_${PROJECT_NAME})
