cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(simplification-example)

add_executable(simplification simplification.cpp)
target_link_libraries(simplification visualization ${GMP_LIBRARIES})
